# ROS / 导航术语外号表

> **用法：** 不必硬背英文缩写。日常用**中文外号 + 一句话**思考；面试前扫一眼英文列即可。
> 配合：[`面试口述-分阶段.md`](面试口述-分阶段.md) · 各阶段复习教材

**版本：** 2026-09-02（阶段 2–4 已学内容）

---

## 阶段 2 · 通信与节点

| 英文 / 符号 | 中文外号 | 一句话 |
|---|---|---|
| **Topic** | 广播 | 持续发数据，不管谁听（`/scan`、`/odom`、图像） |
| **Service** | 打电话 | 问一次、等一次答（查状态） |
| **Action** | 点外卖 | 下单→进度→结果，可取消（导航、巡检任务） |
| **Node** | 节点 | ROS 里一个进程/程序单元 |
| **spin** | 事件循环 | 等回调被调用，别在回调里干重活 |
| **callback** | 回调 | 消息到了自动调用的函数 |
| **QoS** | 收发约定 | 发布订阅要配对（可靠性、历史深度） |
| **TF** | 坐标树 | 各 frame 之间谁相对谁、差多少 |
| **frame** | 坐标系 | 如 `map`、`odom`、`base_footprint` |
| **Static TF** | 静态坐标 | 不变的边（迟到订阅也能拿到） |
| **declare_parameter** | 声明参数 | 不接外部 yaml 往往吃不到配置 |
| **launch** | 装配清单 | 一键起多个节点、灌参数 |
| **rosidl** | 接口编译器 | `.msg/.srv/.action` 生成 Python/C++ 代码 |
| **inspection_interfaces** | 接口契约包 | 全项目消息格式先定在这里 |

---

## 阶段 3 · 模型与仿真

| 英文 / 符号 | 中文外号 | 一句话 |
|---|---|---|
| **URDF** | 机器人说明书 | link/joint 描述形状与装配 |
| **xacro** | 带参数的 URDF | `${}` 引用，改一处连锁 |
| **link** | 零件 | 车身、轮子、雷达等 |
| **joint** | 关节 | 零件怎么连在一起、能不能转 |
| **visual** | 外观 | 好看，不影响物理 |
| **collision** | 碰撞体 | 物理引擎撞到什么形状 |
| **inertial** | 惯性 | 质量+转动惯量，没有会翻车穿地 |
| **base_footprint** | 地上锚点 | 导航二维原点，在地面 z=0 |
| **base_link** | 车身中心 | 离地约 0.10 m，装传感器用 |
| **continuous** | 无限转关节 | 轮子用；revolute 有角度限位 |
| **diff_drive** | 差速插件 | 收速度驱轮，读轮造 `/odom` |
| **cmd_vel** | 速度命令 | Twist：前进线速度 + 转角速度 |
| **cmd_vel_timeout** | 超时刹车 | 0.5 s 无新指令→发零速 |
| **cmd_vel_gazebo** | 插件入口 | diff_drive 实际订阅的话题 |
| **odom** | 轮子里程 | 轮子积分，短期顺、长期漂 |
| **p3d / ground_truth** | 仿真尺子 | 物理引擎直读真位姿，真机没有 |
| **joint_states** | 关节角度 | 轮子转多少度，喂 TF |
| **plugin** | Gazebo 插件 | 焊在仿真里，桥接 ROS 话题 |
| **mu** | 摩擦系数 | 驱动轮抓地 1.0，万向轮滑 0.1 |
| **ray sensor** | 激光仿真 | Gazebo 里发射线测距 |
| **LaserScan** | 激光消息 | `/scan` 的类型，一圈距离数组 |
| **min_range** | 最近有效距离 | 比这近的命中变 inf；由车尺寸定 |
| **marking / clearing** | 画障 / 擦障 | 激光打到涂黑，穿过擦掉 |
| **wheel_separation** | 轮距 | 填进 diff_drive，错则转弯里程计偏 |
| **wheel_diameter** | 轮径 | 填进 diff_drive，错则直线里程计偏 |
| **scale（T3）** | 路程比值 | odom 路程÷真值路程；1.0=机械参数对 |
| **camera_optical_frame** | 光学系 | Z 前 X 右 Y 下；像素投影用 |
| **HFOV** | 水平视场角 | 相机看多宽，影响像素密度 |
| **pitch / roll / yaw** | 俯仰/横滚/航向 | 绕 Y 轴转=pitch（抬头低头） |

---

## 阶段 4 · 导航栈

| 英文 / 符号 | 中文外号 | 一句话 |
|---|---|---|
| **Nav2** | 导航栈 | 规划+控制+自救+行为树一整套 |
| **SLAM** | 建图 | 边走边画地图 → `test_room.pgm` |
| **AMCL** | 地图定位 | 激光+地图猜「我在哪」，发 map→odom |
| **map** | 地图坐标 | SLAM 冻住的墙，原点固定 |
| **map→odom** | 定位图钉 | AMCL 修的 TF，可能跳变 |
| **initialpose** | 初始位姿 | 告诉 AMCL 从哪开始猜 |
| **particle_cloud** | 粒子云 | AMCL 不确定性可视化 |
| **alpha（odom_alpha）** | 运动噪声 | 越大粒子散得越厉害；你们 0.05 |
| **costmap** | 障碍格子 | 涂黑=不能走，晕圈=最好别走 |
| **local costmap** | 眼前格子 | 3×3 m，**odom** 系，给 DWB |
| **global costmap** | 全图格子 | 整张 **map**，给算路 |
| **static_layer** | 静态墙层 | 把 pgm 地图铺进 global |
| **obstacle_layer** | 动态障碍层 | 激光 marking/clearing |
| **inflation_layer** | 障碍晕圈 | 离墙远点，软代价非硬墙 |
| **footprint** | 车身占地 | 0.4×0.3 多边形致命区 |
| **inflation_radius** | 晕圈半径 | 0.35；影响过门舒适度 |
| **planner** | 算路员 | 在 global 上算 A→B 路径（NavFn） |
| **NavFn** | 泛洪算路器 | 栅格代价场上 Dijkstra 式泛洪，再沿代价下坡 |
| **navigation function** | 导航势场 | 每格到终点的最小代价，泛洪算出来的 |
| **Grid / 栅格** | 格子地图 | costmap 5 cm 一格，每格有代价 |
| **Dijkstra / 泛洪** | 水漫算法 | 从终点往外扩，累加代价，经典图搜索 |
| **A\*** | 启发式搜索 | Nav2 可选；你们 `use_astar: false` 用 NavFn 泛洪 |
| **Smac Planner** | 可转向规划器 | Nav2 备选，路径更贴车；小图不必换 |
| **path / global plan** | 全局路径 | 算路员输出的一串路点，给 DWB 跟 |
| **DWB** | 眼前司机 | 20 Hz 试速度、躲障、跟路径 |
| **critics** | 打分规则 | PathDist 嗓门大=最怕偏离路径 |
| **BT / bt_navigator** | 总指挥剧本 | 算路→跟路→失败 recovery |
| **recovery** | 自救 | Spin 转圈、BackUp 后退 |
| **progress_checker** | 进展考官 | 10 s 没走出半径→判卡住 |
| **goal_checker** | 到位考官 | xy/yaw 容差判到没到 |
| **xy_goal_tolerance** | 位置容差 | goal_checker 与 FollowPath **两处一致** |
| **velocity_smoother** | 速度磨平 | 限幅限加速度，最后一道闸 |
| **NavigateToPose** | 单点导航动作 | Nav2 Action，gate/patrol 用它 |
| **ClearEntireCostmap** | 清图服务 | patrol 航点间擦 local/global 障 |
| **allow_unknown** | 允许穿未知 | planner 别太保守绕远 |
| **tolerance（planner）** | 规划容差 | 0.5 m 次优终点暗雷 |
| **rolling_window** | 滚动窗 | local 格子绑在车上走 |
| **reliability / TRANSIENT_LOCAL** | 锁存 QoS | amcl_pose、静态 TF 常用 |

---

## 阶段 5–7 · 预告（学到再背）

| 英文 | 中文外号 | 一句话 |
|---|---|---|
| **Detection** | 检测消息 | YOLO 输出框+类别 |
| **AnomalyEvent** | 异常事件 | 巡检发现的异常上报 |
| **PatrolTask** | 巡检任务 Action | 阶段 6 任务外壳 |
| **rosbridge** | Web 桥 | 浏览器订 ROS 话题 |

---

## 数据流速记（跨阶段）

```text
地图 map + AMCL → map→odom → 定位
/scan → costmap(local odom / global map)
算路员 NavFn：global 栅格泛洪 → 路径 → DWB → smoother → /cmd_vel
```

---

## 相关

- 背诵：[`面试口述-分阶段.md`](面试口述-分阶段.md)
- 课程索引：[`README.md`](README.md)
