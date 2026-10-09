#!/usr/bin/env python3
"""ROS 2 image pipeline for YOLO detection.

The subscription callback keeps only the newest frame. Inference runs in a
worker thread so a slow model cannot build an unbounded image backlog.
With an empty model_path the node still exercises cv_bridge and publishes an
annotated passthrough image plus an empty DetectionArray.
"""

from __future__ import annotations

import queue
import threading
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from inspection_interfaces.msg import Detection, DetectionArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/vision/detections')
        self.declare_parameter('annotated_topic', '/vision/annotated')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('queue_size', 1)

        image_topic = str(self.get_parameter('image_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        annotated_topic = str(self.get_parameter('annotated_topic').value)
        model_path = str(self.get_parameter('model_path').value).strip()
        queue_size = max(1, int(self.get_parameter('queue_size').value))

        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value,
        )
        self._iou_threshold = float(self.get_parameter('iou_threshold').value)
        self._device = str(self.get_parameter('device').value)
        self._bridge = CvBridge()
        self._frames: queue.Queue[tuple[Header, np.ndarray]] = queue.Queue(
            maxsize=queue_size,
        )
        self._stop_event = threading.Event()
        self._model = self._load_model(model_path)

        self._detections_pub = self.create_publisher(
            DetectionArray, detections_topic, 10,
        )
        self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10,
        )
        self._worker = threading.Thread(
            target=self._inference_loop,
            name='yolo-inference',
            daemon=True,
        )
        self._worker.start()

        backend = 'ultralytics' if self._model is not None else 'passthrough'
        self.get_logger().info(
            f'YOLO detector ready: image={image_topic} '
            f'detections={detections_topic} annotated={annotated_topic} '
            f'backend={backend}',
        )

    def _load_model(self, model_path: str) -> Any | None:
        if not model_path:
            self.get_logger().warn(
                'model_path is empty; running image pipeline in passthrough mode',
            )
            return None

        resolved = Path(model_path).expanduser()
        if not resolved.exists():
            raise FileNotFoundError(f'YOLO model does not exist: {resolved}')

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'model_path was set but ultralytics is not installed',
            ) from exc
        self.get_logger().info(f'loading YOLO model: {resolved}')
        return YOLO(str(resolved))

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        item = (msg.header, frame)
        try:
            self._frames.put_nowait(item)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frames.put_nowait(item)
            except queue.Full:
                pass

    def _inference_loop(self) -> None:
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                header, frame = self._frames.get(timeout=0.1)
            except queue.Empty:
                continue

            detections: list[Detection] = []
            annotated = frame.copy()
            if self._model is not None:
                try:
                    detections, annotated = self._infer(frame, header)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f'YOLO inference failed: {exc}')

            detection_array = DetectionArray()
            detection_array.header = header
            detection_array.detections = detections
            self._detections_pub.publish(detection_array)

            try:
                image_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                image_msg.header = header
                self._annotated_pub.publish(image_msg)
            except CvBridgeError as exc:
                self.get_logger().error(f'annotated image conversion failed: {exc}')

    def _infer(
        self,
        frame: np.ndarray,
        header: Header,
    ) -> tuple[list[Detection], np.ndarray]:
        results = self._model.predict(
            source=frame,
            conf=self._confidence_threshold,
            iou=self._iou_threshold,
            device=self._device,
            verbose=False,
        )
        result = results[0]
        annotated = frame.copy()
        detections: list[Detection] = []
        names = result.names

        if result.boxes is None:
            return detections, annotated

        for box in result.boxes:
            coordinates = box.xyxy[0].detach().cpu().tolist()
            confidence = float(box.conf[0].detach().cpu().item())
            class_id = int(box.cls[0].detach().cpu().item())
            class_name = str(names[class_id])
            x1, y1, x2, y2 = (float(value) for value in coordinates)

            detection = Detection()
            detection.header = header
            detection.class_name = class_name
            detection.confidence = confidence
            detection.x1 = x1
            detection.y1 = y1
            detection.x2 = x2
            detection.y2 = y2
            detection.track_id = -1
            detections.append(detection)

            cv2.rectangle(
                annotated,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                annotated,
                f'{class_name} {confidence:.2f}',
                (int(x1), max(0, int(y1) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        return detections, annotated

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
