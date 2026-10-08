# 咨询简报：请重新设计 B→C 机动航点（test_room / Nav2 Humble）

> **用途：** 复制本文件给其他 AI / 同事，请他们**重新给出可执行的 world 坐标机动点**（或否定「拆点」并给出替代）。  
> **仓库：** `https://github.com/soul0109/-YOLO-ROS2-`（分支 `main`，相关 commit `ef457e8`）  
> **环境：** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11（VMware）  
> **日期：** 2026-10-08  
> **请勿建议：** 换 Jazzy/24.04 作主路径；未看几何就大改 Smac/全参；用「每站硬 initialpose」当最终架构。

---

## 1. 要解决什么

规格巡检顺序：**充电位 → A → B → C**（语义三柜点）。  
压力测试显示：**A→B 几乎永远干净；不稳定几乎全在 B→C**（高 `number_of_recoveries`，偶发 ABORT）。

工程共识（已对齐多方）：

- 主线是 **巡检任务建模**（语义点 vs 机动点），不是先删 Spin / 狂调 Progress。  
- Phase1 实验：只改 B→C，A→B 不动，做 A/B 对照。  
- **当前内部实现的机动点坐标被质疑失败**——请外部重新设计点位（或指出测试无效）。

---

## 2. 场景几何（world，单位 m）

地图 origin ≈ `(-0.0134, 0.0519)`；Nav2 goal 用 map = world − origin。  
整体：约 10×6 三机房 + 走廊。

### 2.1 房间与门（均门宽 1.1 m）

| 房间 | 大致范围 | 门墙 y | 门洞中心 x | 门朝向 |
|---|---|---|---|---|
| A（西南） | x≈1.7–4.6, y≈0–2.25 | 2.25 | ≈3.15 | 朝北进走廊 |
| B（东南） | x≈7.0–9.9, y≈0–2.25 | 2.25 | ≈8.45 | 朝北进走廊 |
| C（北中） | x≈3.9–6.6, y≈3.75–5.9 | 3.75 | ≈5.25 | 朝南进走廊 |

走廊带：大致 **y ∈ (2.25, 3.75)**。

房 B 北墙门洞细算（供验算）：

- `wall_b_north_L`：center x=7.475, size_x=0.85 → 约占 [7.05, 7.90]  
- `wall_b_north_R`：center x=9.475, size_x=0.95 → 约占 [9.00, 9.95]  
- **门洞开口 ≈ x∈[7.90, 9.00]**，中心 **8.45**，墙厚 0.1 @ y=2.25  

房 C 南墙门洞：

- L：4.275±0.425 → [3.85, 4.70]；R：6.225±0.425 → [5.80, 6.65]  
- **门洞 ≈ x∈[4.70, 5.80]**，中心 **5.25** @ y=3.75  

### 2.2 设备柜与巡检点（冻结，勿改语义）

| 对象 | world (x,y) | yaw | 说明 |
|---|---|---|---|
| cabinet_b | (8.45, 0.35) | — | 房 B 南侧柜 |
| **巡检 B** | **(8.45, 1.75)** | **−π/2（朝南对柜）** | 停车标记同此 |
| cabinet_c | (5.25, 5.65) | — | 房 C 北侧柜 |
| **巡检 C** | **(5.25, 4.25)** | **+π/2（朝北对柜）** | 停车标记同此 |
| 巡检 A | (3.15, 1.75) | −π/2 | 对称于 B |

机器人：差速 footprint **0.4×0.3 m**；巡航 **max_vel_x≈0.15 m/s**；  
costmap `inflation_radius=0.35`；Nav2 GoalChecker **xy=0.10 m, yaw=0.10 rad**（机动点 Phase1 **未**换宽松 checker）。

---

## 3. 已跑通 / 失败的实验证据

### 3.1 基线 direct：单 goal `B → C`（无中间点）

目录：`bags/patrol_bc_fast_direct_20261008_142240/`  
流程：每轮 delete+spawn@B + 硬 `/initialpose`(B) + `NavigateToPose(C)`  

| 结果 | recoveries（PASS 轮，日志） |
|---|---|
| **PASS 8 / 10** | 1,4,9,1,2,6,5,3（FAIL 两轮无完整 diag） |

说明：**几何上 B→C 常常能到**，但 recovery 偏高；不是「门完全过不去」。

### 3.2 内部设计 v1（已证明坏）

```text
B (8.45,1.75,-90°) → B_egress(8.45,1.90,+90°) → C_approach(5.25,3.00,+90°) → C
```

| 现象 | 证据 |
|---|---|
| 出不去 / 撞墙感 | 柜前仅平移 0.15 m 却要求 yaw +180° → **原地拧** |
| B_egress Timeout 120s / ABORT | `bags/patrol_bc_fast_split_20261008_143830/` |
| FAIL 后仍跑后续段 | 脚本缺陷（已修：首段 FAIL 即停） |

**结论：v1 点位设计错误（机动点叠在巡检点旁）。**

### 3.3 内部设计 v2（仍 FAIL；且首轮定位可疑）

```text
B → B_egress(8.45, 2.55, 180°西) → C_approach(5.25, 3.00, +90°) → C
```

意图：把门外出到走廊，车头朝西对准去 C 的方向。

首轮日志 `bags/patrol_bc_fast_split_20261008_144923/run01.log`：

```text
initialpose 声称: world B (8.45, 1.75, -90°)
AMCL_CHECK: xy=(3.429, 1.859)  var_xy=(0.715,0.951)  var_yaw=1.215
→ NavigateToPose(B_egress 8.45,2.55,π) → ABORTED (rec=0)
```

**关键：发了 B 的 initialpose 后，AMCL 却在 ≈(3.4,1.9)，协方差爆。**  
旧校验只查「有限数」，**不查是否靠近 B**。  
因此：**不能把本轮 FAIL 单独归因于 (8.45,2.55) 点位错误**——也可能是 spawn/AMCL 未收敛就开导。  
但 v1 的几何错误是实锤；v2 是否可用 **尚未在「AMCL 锁在 B」条件下被公正验证**。

---

## 4. 当前代码里的点（供对照，可全部推翻）

文件：`ros2_ws/src/inspection_mission/inspection_mission/patrol_mission_node.py`  
脚本：`scripts/patrol_bc_fast.sh`（`direct` | `split`）

| name | world x,y | yaw | kind |
|---|---|---|---|
| A | 3.15, 1.75 | −π/2 | inspect（本实验不测） |
| B | 8.45, 1.75 | −π/2 | inspect |
| B_egress | 8.45, 2.55 | π | maneuver（v2） |
| C_approach | 5.25, 3.00 | +π/2 | maneuver |
| C | 5.25, 4.25 | +π/2 | inspect |

A→B 仍直达，无 A_egress。

---

## 5. 约束与验收（请按此交答案）

### 必须遵守

1. **不要改**巡检语义点 A/B/C 的 world 坐标与柜朝向（场景规格冻结）。  
2. 机动点可以增删改；建议仍用 **串行 NavigateToPose**（便于分段诊断）。  
3. Phase1 暂不改：AMCL alpha、ProgressChecker、Spin/BT、planner 类型、inflation（除非你证明门被膨胀堵死）。  
4. 机动点在现有 GoalChecker（xy0.10 / yaw0.10）下也要 **可 SUCCEEDED**——或明确要求「必须先上宽松 GoalChecker」，并给出阈值。  
5. 主环境保持 Humble + Gazebo Classic。

### 请你交付

1. **推荐的 B→C 航段列表**（name, world_x, world_y, yaw_rad, 一句话职责）。  
2. 每个点为什么在那里（相对门中心 / 走廊中线 / 柜的距离）。  
3. 在 RViz/脑子里走一遍：从 B 朝南 → 出 B 门 → 走廊 → 进 C 门 → 对柜，差速 0.15 m/s 是否避免「窄处 180°」。  
4. 是否需要 2 段 / 3 段 / 4 段；**反对**只为了好看硬拆。  
5. 测试协议：如何判定 AMCL 已锁在 B（建议：\|AMCL−B\|&lt;0.25 m 且 var_yaw&lt;0.2）再发第一段。  
6. 若你认为 **不该拆点**，说明用什么替代（Rotation Shim / 路径约束等），并给最小验证实验。

### 成功判据（实验）

在 **AMCL 已锁 B** 前提下：

```bash
bash scripts/patrol_bc_fast.sh 10 split   # 使用你的点
```

对比已有 direct（8/10，rec 常见 1–9）：

- split PASS ≥ direct；  
- 各段 recoveries 总和明显下降（目标体感：常双位数 → 多数轮 0–2）。

---

## 6. 请直接回答的问题

1. v2 的 `B_egress=(8.45,2.55,π)` 在几何上是否合理？若否，给出替换坐标。  
2. `C_approach=(5.25,3.00,+π/2)` 是否太靠近/太远 C 门？  
3. 从背对门口的柜前点出发，**第一段**应是「房内先转到朝门」还是「直接 goal 到门外走廊点让 Nav2 自己弧线」？  
4. 在 GoalChecker 仍严格时，机动点 yaw 是否应故意与路径切向一致、避免到点再大转？  
5. 本次 AMCL 飞到 (3.4,1.9) —— 你建议测试脚本如何硬门禁，避免假阴性？

---

## 7. 关键文件路径

| 路径 | 内容 |
|---|---|
| `ros2_ws/src/simulation_worlds/worlds/test_room.world` | 墙/门/柜几何 |
| `ros2_ws/src/navigation_config/maps/test_room.{pgm,yaml}` | 占用栅格 |
| `ros2_ws/src/navigation_config/config/nav2_test_room.yaml` | Nav2 参数 |
| `ros2_ws/src/inspection_mission/.../patrol_mission_node.py` | 当前航点表 |
| `scripts/patrol_bc_fast.sh` | direct/split 探针 |
| `docs/巡检场景规格.md` | 到点闸门数字 |
| `bags/patrol_bc_fast_direct_20261008_142240/` | direct 基线 |
| `bags/patrol_bc_fast_split_20261008_143830/` | v1 坏点证据 |
| `bags/patrol_bc_fast_split_20261008_144923/` | v2 首轮（含 AMCL 飞点） |

---

## 8. 一句话求助

请基于 **test_room 真实门洞坐标**，重新设计 **B→C 的机动航点序列**（或论证不拆点），并给出可粘贴进 `WAYPOINTS` 的数字；同时给出 **「AMCL 已在 B」** 的测试门禁，避免再把定位失败误判成航点失败。
