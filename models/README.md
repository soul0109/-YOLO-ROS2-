# Models

存放 YOLO 权重、导出的 ONNX、以及训练相关说明。

**不要把大文件提交进 Git。** `.gitignore` 已忽略 `*.pt` / `*.onnx` / `*.engine` 与 `datasets/`。

## 约定

| 路径 | 用途 |
|---|---|
| `weights/` | 发布用权重（本地放置，本文件写版本与哈希） |
| `export/` | ONNX / TensorRT 导出产物 |
| `datasets/` | 训练集（本地或网盘） |
| `train/` | 训练脚本与超参 |

## M1 固定权重（2026-10-10）

| 项 | 值 |
|---|---|
| 文件 | `weights/yolov8n.pt` |
| 来源 | Ultralytics 官方 yolov8n（COCO 预训练） |
| SHA256 | `f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36` |
| 用途 | **仅**验证 ROS 推理链路（阶段 5A）；**不能**证明巡检灯/面板检出达标 |

下载（GitHub release 直连慢/403 时可用镜像）：

```bash
mkdir -p ~/inspection-robot/models/weights && cd $_
curl -fL -o yolov8n.pt \
  'https://ghfast.top/https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt'
sha256sum -c yolov8n.pt.sha256
```

节点配置：`ros2_ws/src/vision_perception/config/yolo_detector.yaml` → `model_path`。

## 运行时依赖（VM）

```bash
python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple \
  'numpy>=1.23,<2' 'ultralytics>=8.3,<9'
# torch CPU（若尚未安装）按 pytorch.org 说明；勿让 pip opencv 覆盖系统 cv2 导致 cv_bridge 崩
# 若误装 opencv-python / numpy 2.x：卸载 opencv-python，并 pip install 'numpy<2'
```

**已知坑：** Humble `cv_bridge` 与 **NumPy 2.x** 不兼容（`_ARRAY_API not found`）。保持 `numpy<2`，优先用 apt 的 `python3-opencv`。

## 冒烟（开发验证，非完整测试框架）

```bash
bash ~/inspection-robot/scripts/yolo_m1_smoke.sh
# 证据目录：bags/vision_m1_smoke_*
```

M2/5B 的场景测试集与 90% 口径另立，不在此提前展开。
