# Models

存放 YOLO 权重、导出的 ONNX、以及训练相关说明。

**不要把大文件提交进 Git。** `.gitignore` 已忽略 `*.pt` / `*.onnx` / `*.engine` 与 `datasets/`。

## 约定

| 路径 | 用途 |
|---|---|
| `weights/` | 发布用权重（本地放置，README 写下载方式） |
| `export/` | ONNX / TensorRT 导出产物 |
| `datasets/` | 训练集（本地或网盘） |
| `train/` | 训练脚本与超参 |

## 起步模型

阶段 4 先用 Ultralytics 官方 `yolov8n.pt`（COCO person）跑通链路，再自训练巡检类别。
