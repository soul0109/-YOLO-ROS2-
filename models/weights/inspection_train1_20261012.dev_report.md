# Train-1 · 开发集训练报告（Windows GPU）

> **开发 val ≠ 正式 5B test。** 禁止据此宣称正式准确率 / 90%。未写 `test_batches.txt`，未冻 F1。

## 环境

| 项 | 值 |
|---|---|
| Host | Windows 11 · RTX 3060 Ti 8GB · CUDA driver 13.2 |
| device | `0` → `cuda:0` (NVIDIA GeForce RTX 3060 Ti) |
| ultralytics | 8.4.175 |
| torch | 2.5.1+cu121 |
| Python | 3.12.10 |
| 初始化权重 | `models/weights/yolov8n.pt` |
| init sha256 | `f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36` |

## 实际命令

中文仓库路径会导致 ultralytics 解析异常；本机用 `subst Y:` 映射仓库根后执行（`amp=False`：避免 AMP 检查去 GitHub 拉 `yolo26n.pt` 超时）。

```text
subst Y: <repo_root>
yolo settings datasets_dir=Y:\bags\vision_5b_c2\yolo_dev
yolo detect train data=Y:\bags\vision_5b_c2\yolo_dev\data.yaml model=Y:\models\weights\yolov8n.pt epochs=50 imgsz=640 batch=16 device=0 project=Y:\bags\vision_5b_c2 name=train1 exist_ok=True plots=True amp=False workers=4
```

等价意图（菜谱原命令）：`epochs=50 imgsz=640 batch=16 device=0`，产物落在 `bags/vision_5b_c2/train1/`。

## 数据

- 划分：soft / 非正式 · train **81** / val **25** · 无 aug
- 三类：`indicator_light` / `panel` / `inspection_region`
- `data.yaml` note: `dev_only_not_formal_5b_test`

## 产物

| 路径 | 说明 |
|---|---|
| `bags/vision_5b_c2/train1/weights/best.pt` | 最佳权重 |
| `bags/vision_5b_c2/train1/weights/last.pt` | 末轮权重 |
| `bags/vision_5b_c2/train1/args.yaml` | 超参 |
| `bags/vision_5b_c2/train1/results.csv` | 逐 epoch 指标 |
| `bags/vision_5b_c2/train1/results.png` | 曲线图 |
| `confusion_matrix*.png` / `val_batch*.jpg` | 有则保留 |

**best.pt sha256：** `a1d2507a6fe5fc9bae12e85f28fc3e876477b010b88640bb251d198ee023fed4`

训练墙钟约 **0.088 h**（~5.3 min，清掉重复进程后）。

## 开发 val 指标（best.pt 最终验证）

> 仅 25 张 soft val，**极度乐观、不可当正式验收**。数值原样记录，不作 90% 宣称。

| Class | Images | Instances | P | R | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|---:|
| all | 25 | 125 | 0.998 | 1.000 | 0.995 | 0.995 |
| indicator_light | 25 | 75 | 0.999 | 1.000 | 0.995 | 0.995 |
| panel | 25 | 25 | 0.996 | 1.000 | 0.995 | 0.995 |
| inspection_region | 25 | 25 | 0.998 | 1.000 | 0.995 | 0.995 |

## 明确未做

正式 5B test 冻结 / `test_batches.txt` / 90% 宣称 / Web / Nav2 / 再采 / 改 VM ROS·Gazebo。

## VM 侧下一步

```bash
cd ~/inspection-robot
ls -la bags/vision_5b_c2/train1/weights/best.pt
python3 scripts/hook1_yolo_roi_e1.py --weights bags/vision_5b_c2/train1/weights/best.pt
```
