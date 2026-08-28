# 阶段 2：ROS2 通信 demo 说明文档

> 面向初学者：说明阶段 2 代码做了什么、文件在哪、如何在 VM 里编译验收。  
> 配套代码：`ros2_ws/src/inspection_demos/`、`inspection_interfaces/srv/RobotStatus.srv`

**最后更新：** 2026-08-28

---

## 1. 阶段 2 目标（一句话）

在没有 Gazebo 和真机器人的情况下，用**假数据**跑通 ROS2 的四种基本通信方式：**Topic、Service、Action、TF**。

---

## 2. 整体架构

```text
                    demos.launch.py（一键启动）
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
  log_publisher          robot_status_server    patrol_action_server
        │                     │                     │
        │ /inspection/log     │ /get_robot_status   │ /patrol_task
        ▼                     ▼                     ▼
  log_subscriber          （命令行调用）          （命令行/客户端调用）

  static_tf_broadcaster
        │
        └── TF: base_link → camera_link
```

| 通信类型 | 节点 | 接口名 | 消息/服务类型 |
|---|---|---|---|
| Topic 发布 | `log_publisher` | `/inspection/log` | `std_msgs/String` |
| Topic 订阅 | `log_subscriber` | `/inspection/log` | `std_msgs/String` |
| Service | `robot_status_server` | `/get_robot_status` | `inspection_interfaces/srv/RobotStatus` |
| Action | `patrol_action_server` | `/patrol_task` | `inspection_interfaces/action/PatrolTask` |
| TF 静态变换 | `static_tf_broadcaster` | TF 树 | `base_link` → `camera_link` |

---

## 3. 新增/修改的文件

### 3.1 协议包 `inspection_interfaces`

| 文件 | 作用 |
|---|---|
| `srv/RobotStatus.srv` | 定义「查机器人状态」的请求/响应格式 |
| `CMakeLists.txt` | 增加 srv 编译项 |

**RobotStatus 字段说明：**

```text
请求：（空）
响应：
  robot_id          机器人编号，如 robot_01
  status            idle | patrolling | error
  battery_percent   电量 0~100
  pose              机器人在地图中的位姿（geometry_msgs/Pose）
```

### 3.2 功能包 `inspection_demos`（新建）

```
inspection_demos/
├── inspection_demos/          # Python 源码（每个文件一个节点）
│   ├── log_publisher.py       # 定时发布巡检日志
│   ├── log_subscriber.py      # 订阅并打印日志
│   ├── robot_status_server.py # 状态查询服务
│   ├── patrol_action_server.py# 模拟巡检 Action
│   └── static_tf_broadcaster.py # 相机静态 TF
├── config/demos.yaml          # 全部节点参数（可被 launch 加载）
├── launch/demos.launch.py     # 一键启动
├── package.xml
├── setup.py                   # 注册 5 个可执行入口
└── setup.cfg
```

---

## 4. 各节点代码做了什么

### 4.1 `log_publisher.py` — Topic 发布者

- 每 2 秒（可配置）向 `/inspection/log` 发一条字符串。
- 内容从预置的 6 条巡检日志里循环取，前面加上 `[robot_01]` 前缀。
- **关键 API**：`create_publisher`、`create_timer`、`publish`。

### 4.2 `log_subscriber.py` — Topic 订阅者

- 订阅 `/inspection/log`，收到后在终端打印 `[订阅收到] ...`。
- **关键 API**：`create_subscription`、回调函数。
- 与 publisher 是两个独立进程，只靠**话题名 + 消息类型**连接。

### 4.3 `robot_status_server.py` — Service 服务端

- 提供 `/get_robot_status` 服务；客户端调用时返回 YAML 里配置的假状态。
- 默认：`idle`、电量 85%、位置 (1.5, 2.0, 0.0)。
- **关键 API**：`create_service`、在回调里填充 `response` 并返回。

### 4.4 `patrol_action_server.py` — Action 服务端

- 提供 `/patrol_task` Action（接口定义在 `PatrolTask.action`，阶段 1 已有）。
- 收到航点列表后，**模拟**逐个前往：
  1. `phase=navigating`，发 feedback（当前航点、进度百分比）
  2. 若 `snapshot_on_arrive=true`，再模拟拍照阶段
  3. 全部完成后 `succeed`，返回 `visited_count` 等
- 支持取消（`cancel`）。
- **关键 API**：`ActionServer`、`publish_feedback`、`goal_handle.succeed()`。

### 4.5 `static_tf_broadcaster.py` — 静态坐标变换

- 广播：`base_link`（父）→ `camera_link`（子）。
- 默认相机安装在主体前方 0.10m、上方 0.30m。
- **关键 API**：`StaticTransformBroadcaster`、`TransformStamped`。
- 阶段 3 有真机器人后，这条 TF 会随 URDF 一起由 robot_state_publisher 管理；阶段 2 先手写理解概念。

### 4.6 `config/demos.yaml` — 参数文件

- 把「发布间隔、机器人 ID、电量、相机位置」等从代码里拆出来。
- launch 按**节点名**加载对应段落，改参数不用改 Python。

### 4.7 `launch/demos.launch.py` — 一键启动

- 同时启动上面 5 个节点，并加载 `demos.yaml`。
- 企业项目里通常用 launch 编排多个节点，而不是手动开 5 个终端。

---

## 5. 编译与运行（Ubuntu VM）

### 5.1 同步代码（宿主机改完后）

```bash
bash ~/inspection-robot/scripts/sync_from_share.sh
```

### 5.2 编译

```bash
cd ~/inspection-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select inspection_interfaces inspection_demos --symlink-install
source install/setup.bash
```

> 必须先编 `inspection_interfaces`（含新 srv），再编 `inspection_demos`。

### 5.3 一键启动

```bash
ros2 launch inspection_demos demos.launch.py
```

应看到 5 个节点启动，终端每隔约 2 秒有日志发布/订阅输出。

---

## 6. 验收命令清单（另开终端，先 source）

```bash
source /opt/ros/humble/setup.bash
source ~/inspection-robot/ros2_ws/install/setup.bash
```

### 6.1 Topic

```bash
# 查看话题列表
ros2 topic list | grep inspection

# 实时查看日志（应每 2 秒一条）
ros2 topic echo /inspection/log
```

### 6.2 Service

```bash
# 查看服务类型
ros2 service type /get_robot_status

# 调用服务（请求为空 {}）
ros2 service call /get_robot_status inspection_interfaces/srv/RobotStatus {}
```

预期响应含 `robot_id: robot_01`、`status: idle`、`battery_percent: 85.0`。

### 6.3 Action

```bash
# 查看 action 列表
ros2 action list

# 发送一个 2 航点的模拟巡检任务
ros2 action send_goal /patrol_task inspection_interfaces/action/PatrolTask \
"{waypoints: [{header: {frame_id: 'room_1'}}, {header: {frame_id: 'room_2'}}], snapshot_on_arrive: true, task_label: 'demo_patrol'}" \
--feedback
```

预期：终端持续打印 feedback（`phase`、`progress`），最后返回 `success: true`。

### 6.4 TF

```bash
# 查看 base_link 到 camera_link 的变换
ros2 run tf2_ros tf2_echo base_link camera_link
```

预期：Translation 约 `x=0.10, y=0.0, z=0.30`。

### 6.5 节点与参数

```bash
ros2 node list
ros2 param list /log_publisher
ros2 param get /log_publisher log_interval_sec
```

---

## 7. 阶段 2 完成标准（勾选）

- [ ] `colcon build` 无报错
- [ ] `ros2 launch inspection_demos demos.launch.py` 能启动 5 个节点
- [ ] `ros2 topic echo /inspection/log` 有持续输出
- [ ] `ros2 service call /get_robot_status ...` 有结构化返回
- [ ] `ros2 action send_goal /patrol_task ... --feedback` 能看到进度并最终成功
- [ ] `ros2 run tf2_ros tf2_echo base_link camera_link` 有正确平移
- [ ] 能用自己的话解释：Topic / Service / Action / TF 各是什么

全部勾选后，在 `docs/当前进度.md` 把阶段 2 标为完成，再进入阶段 3（`robot_v0`）。

---

## 8. 初学者建议：怎么读这些代码

推荐阅读顺序（每读一个就单独运行验证）：

1. `log_publisher.py` + `log_subscriber.py`（最简单）
2. `robot_status_server.py`
3. `static_tf_broadcaster.py`
4. `patrol_action_server.py`（最复杂，放最后）
5. `demos.yaml` + `demos.launch.py`（理解工程化组织）

每个 `.py` 文件顶部有模块说明，函数内有中文注释，对照 `ros2 topic/service/action` 命令一起看效果最好。

---

## 9. 与后续阶段的关系

| 阶段 2 练到的 | 后面用在哪 |
|---|---|
| `/inspection/log` topic | 任务节点发巡检事件、Web 订阅展示 |
| `RobotStatus.srv` | Web/API 查机器人状态 |
| `PatrolTask` action | 阶段 6 真巡检任务（调 Nav2） |
| `base_link→camera_link` TF | 阶段 5/6 检测框映射到地图 |

阶段 2 的代码**不会被扔掉**，而是逐步换成真数据源；通信模式和接口名尽量保持不变。

---

## 10. 常见问题

**Q: 编译报 `inspection_interfaces.srv` 找不到？**  
A: 先单独编 `inspection_interfaces`，`source install/setup.bash` 后再编 `inspection_demos`。

**Q: launch 起来但没有日志？**  
A: 检查是否 source 了工作空间；`ros2 node list` 看 `log_publisher` 是否在。

**Q: tf2_echo 报 Lookup would require extrapolation？**  
A: 确认 `static_tf_broadcaster` 节点在运行；静态 TF 只需发布一次即可。

**Q: 能否只跑某一个节点？**  
A: 可以，例如：`ros2 run inspection_demos log_publisher --ros-args --params-file <path>/demos.yaml`
