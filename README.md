# 基于 YOLO + ROS2 的智能巡检机器人仿真系统

端到端可运行的智能巡检机器人仿真系统：Gazebo 厂区场景 + SLAM/Nav2 自主巡检 + YOLO 异常检测 + Web 监控平台。

> 当前进度：[`docs/当前进度.md`](docs/当前进度.md) · 执行手册：[`docs/后续开发计划与建模攻关方案.md`](docs/后续开发计划与建模攻关方案.md) · 总蓝图：[`docs/项目执行计划.md`](docs/项目执行计划.md)

## 技术栈

| 层 | 选型 |
|---|---|
| OS / 中间件 | Ubuntu 22.04 + ROS2 Humble |
| 仿真 | Gazebo Classic 11 |
| 导航 | SLAM Toolbox + Nav2 |
| 视觉 | YOLOv8 → ONNX Runtime |
| Web | FastAPI + rosbridge + Vue3 |

## 仓库结构

```
├── docs/                  # 设计文档与周复盘
├── ros2_ws/src/           # ROS2 功能包
│   ├── inspection_interfaces/   # 自定义 msg / srv / action（协议先行）
│   ├── robot_description/       # URDF / xacro
│   ├── simulation_worlds/       # Gazebo 世界与模型
│   ├── vision_perception/       # YOLO 检测与目标定位
│   ├── inspection_mission/      # 巡检任务与异常管理
│   ├── navigation_config/       # Nav2 / SLAM 参数与地图
│   ├── llm_control/             # 自然语言任务（进阶）
│   └── inspection_bringup/      # 一键 launch
├── web/backend/           # FastAPI
├── web/frontend/          # Vue3 Dashboard
├── models/                # 权重与训练说明（大文件不入库）
├── scripts/               # 环境安装与工具脚本
└── assets/                # 演示素材
```

## 快速开始（环境就绪后）

```bash
# 1. 克隆（目录名勿以 - 开头，否则 bash cd 会当成选项）
git clone git@github.com:soul0109/-YOLO-ROS2-.git inspection-robot
cd inspection-robot

# 2. 构建
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash

# 3. 启动（阶段推进后可用）
ros2 launch inspection_bringup bringup.launch.py
```

## 当前进度

详见 [`docs/当前进度.md`](docs/当前进度.md)（活文档）。摘要：

- [x] 阶段 1：ROS2 Humble + Gazebo + `inspection_interfaces` 验收
- [x] 后续开发计划与建模攻关方案
- [ ] **阶段 2（进行中）**：`inspection_demos` + pub/sub / service / action / launch / TF
- [ ] 阶段 3+：见 [`docs/后续开发计划与建模攻关方案.md`](docs/后续开发计划与建模攻关方案.md)

## 开发约定

- 分支：`main`（稳定）+ `dev` + `feature/*`；每阶段打 tag（`v0.1-sim` …）
- 大文件（权重/数据集/rosbag）不入库，见 `models/README.md`
- 提问模板与协作方式见执行计划第八节
