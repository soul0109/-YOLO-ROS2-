# 咨询简报：实验 A（direct B→C）五轮结果——请判读根因

> **用途：** 复制全文给其他 AI。请基于数据判断：问题在 Nav2 路径执行、失败分类缺失、还是仍应先做机动点拆分。  
> **仓库：** `https://github.com/soul0109/-YOLO-ROS2-`（本地 HEAD `7639970`）  
> **环境：** Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11（VMware）  
> **证据目录：** `bags/patrol_bc_fast_direct_20261008_150626/`  
> **日期：** 2026-10-08  
> **请勿建议：** 换 Jazzy/24.04；未分类失败就全面重调 yaml；用每站硬 initialpose 当最终架构。

---

## 0. 先说清楚：这轮测的是什么 / 不是什么

| 项 | 内容 |
|---|---|
| 实验名 | **实验 A（direct）** |
| 目的 | 验证 **AMCL 门禁修好后**，单 goal `B→C` 基线是否仍健康 |
| **不是** | 实验 B（`BC_SPLIT_V3` 多机动点）；本轮 **没有** 跑 B_egress / corridor_turn |
| 流程 | 每轮：`delete+spawn@B` → 硬 `/initialpose`(B) → **AMCL hold~1s 锁在 B** → `NavigateToPose(C)` |
| 用户操作 | 计划 ×10；实际跑完 1–4，第 5 轮导航中 **手动 Ctrl+C 暂停** |

**结论边界：** 这轮只能说明「干净站在 B、门禁通过后，直达 C」的表现；**不能**评价 V3 机动点是否有效（尚未公正开跑）。

---

## 1. 汇总表（请先看这里）

| # | 结果 | AMCL@B | 到位误差 | recoveries | 备注 |
|---|---|---|---|---|---|
| 1 | **PASS** | dist=0.042, var_yaw=0.064, LOCKED | 0.157 m / 5.1° | **6** | 闸门过，但 recovery 偏高 |
| 2 | **PASS** | dist=0.046, var_yaw=0.067, LOCKED | 0.157 m / 5.2° | **10** | 同上，更差 |
| 3 | **FAIL** | dist=0.048, var_yaw=0.061, LOCKED | — | **0**（无 feedback） | `NavigateToPose ABORTED`，几乎无细节 |
| 4 | **PASS** | dist=0.042, var_yaw=0.064, LOCKED | 0.131 m / 4.7° | **2** | 最好的一轮 |
| 5 | **中断** | dist=0.030, var_yaw=0.066, LOCKED | — | — | 用户 KeyboardInterrupt；**不算导航 FAIL** |

**有效导航轮（1–4）：** PASS **3/4**，FAIL **1/4**。  
**INFRA（AMCL 未锁）：** **0**（门禁修复后正常）。

与更早基线（门禁宽松时代）`patrol_bc_fast_direct_20261008_142240`：**8/10 PASS**，PASS 轮 recoveries 约 1–9，模式一致。

---

## 2. 已搞清楚的事

### 2.1 AMCL 门禁已可用（实验 A 的前置目的达成）

五轮全部：

```text
initialpose → AMCL hold=10/10 → AMCL_LOCKED
amcl_world ≈ (8.41~8.43, 1.76~1.79)
distance ≈ 0.03~0.05 m
var_yaw ≈ 0.06
stamps=1   ← 静止时 AMCL 因 update_min_d/a 往往只发 1 个新 stamp（预期行为）
```

此前错误门禁「要求 10 个新 stamp」会导致假 INFRA；已改为「最新位姿合格并保持 ~1s」。  
→ **本轮 FAIL/高 recovery 不能再甩锅给「没锁在 B」。**

### 2.2 单点 B→C 仍然「能到但不健康」

- PASS 时位置/航向闸门都过（≤0.20 m、≤10°）。  
- 但 recoveries：**6 / 10 / 2** —— 与历史「B→C 高 recovery」同一病。  
- 说明：**问题不在测试 harness，而在 B→C 这条机动本身（或 Nav2 recovery 对这条路的反应）。**

### 2.3 失败轮（run3）信息严重不足

```text
AMCL_LOCKED
NavigateToPose C ...
FAIL: NavigateToPose ABORTED
FINAL: FAIL
recoveries=0   # gate 没采到 feedback 里的 recovery 计数
```

**不知道**是：

- `ComputePathToPose` 一上来就失败？  
- `FollowPath` 失败？  
- ProgressChecker？  
- 还是 BT 很快耗尽？

仓库里 **没有** 同步录 `bt_navigator` / `planner_server` / `controller_server` 本轮日志。  
→ 请外部 AI 指出：下一刀应补什么仪器化，而不是先猜改 Spin。

---

## 3. 场景与参数（相关切片）

### 3.1 起终点（world）

| 点 | x | y | yaw |
|---|---|---|---|
| B（起点，每轮 spawn） | 8.45 | 1.75 | −π/2（朝南对柜） |
| C（唯一 goal） | 5.25 | 4.25 | +π/2（朝北对柜） |

路径语义：房 B 内背对北门 → 出 B 门(y=2.25, x≈8.45) → 走廊 → 进 C 门(y=3.75, x≈5.25) → 对柜。

### 3.2 Nav2（未在本实验改）

| 项 | 值 |
|---|---|
| 全局 | NavFn，tolerance 0.5 |
| 局部 | DWB，max_vel_x **0.15** |
| GoalChecker | xy **0.10** m，yaw **0.10** rad |
| ProgressChecker | radius **0.25** m / **10** s |
| inflation | **0.35** m；footprint 0.4×0.3 |
| AMCL alpha | 0.05；`update_min_d=0.05`，`update_min_a=0.10` |
| Recovery | 默认含 Spin / Backup 等 |

### 3.3 已设计但**本轮未测**的 V3 拆点（供对照）

```text
B → B_egress(8.45,1.55,+π/2)
  → B_corridor_turn(8.45,3.00,π)
  → C_approach(5.25,3.00,+π/2)
  → C
```

`WAYPOINT VERSION = BC_SPLIT_V3`（代码已合入；**实验 B 尚未公正跑完**）。

---

## 4. 逐轮关键日志摘录

### run1 PASS（rec=6）

```text
AMCL_LOCKED amcl_world=(8.426,1.785) dist=0.042 var_yaw=0.064
Position Error: 0.157 m PASS
Yaw Error: 5.1 deg PASS
recoveries during nav: 6
FINAL: PASS
```

### run2 PASS（rec=10）

```text
AMCL_LOCKED amcl_world=(8.417,1.783) dist=0.046 var_yaw=0.067
Position Error: 0.157 m PASS
Yaw Error: 5.2 deg PASS
recoveries during nav: 10
FINAL: PASS
```

### run3 FAIL（ABORT, rec=0）

```text
AMCL_LOCKED amcl_world=(8.417,1.785) dist=0.048 var_yaw=0.061
FAIL: NavigateToPose ABORTED
FINAL: FAIL
```

### run4 PASS（rec=2）

```text
AMCL_LOCKED amcl_world=(8.413,1.770) dist=0.042 var_yaw=0.064
Position Error: 0.131 m PASS
Yaw Error: 4.7 deg PASS
recoveries during nav: 2
FINAL: PASS
```

### run5 用户中断（导航进行中）

```text
AMCL_LOCKED ...
NavigateToPose C ...
KeyboardInterrupt / ActionClient take_feedback TypeError（Ctrl+C 副作用）
```

---

## 5. 历史对照（帮助排序根因）

| 证据包 | 内容 | 启示 |
|---|---|---|
| `direct_20261008_142240` | 8/10 PASS，rec 常 1–9 | 直达 C「大多能到、常带病」 |
| `split_143830` v1 | B_egress=(8.45,1.90,+90°) Timeout/ABORT | 柜前原地 180°，点位错误 |
| `split_144923` v2 | B_egress=(8.45,2.55,π) 首轮 ABORT | AMCL 飞到 (3.4,1.9)，**测试无效**；且 2.55 距墙 0.30&lt;inflation0.35 |
| **本包 `direct_150626`** | 门禁全绿；3/4 PASS；rec 2–10；1 次无细节 ABORT | **基线病仍在；仪器化不够** |

---

## 6. 我们内部的分歧（请仲裁）

**阵营 A（任务建模）：** 直达 B→C 把「出门+走廊+进门+180°朝向」压成一个 goal → 应跑实验 B（V3 拆点），看 rec 是否下降。

**阵营 B（先分类失败）：** run3 这种 `ABORT + rec=0` 必须先分清 ComputePath vs FollowPath vs Progress，否则拆点也可能「带着未知病前进」。

**阵营 C（勿先砍 Spin）：** recovery 高是症状放大器；根因可能是路径几何 / 门口执行，不是先删 Spin。

请明确：**下一步唯一动作**应是哪一个（只选一个主路径）。

---

## 7. 请你具体回答

1. 在 AMCL 已锁 B 的前提下，PASS 轮 recoveries=6~10 的最可能机制排序是什么？  
2. run3「ABORT 且 recoveries=0」通常意味着什么？应补采哪些话题/日志字段？  
3. 现在是否仍推荐立刻跑 `bash scripts/patrol_bc_fast.sh 10 split`（V3）？还是先加失败分类再跑？  
4. 若跑 split，验收尺子如何定（PASS 率、分段 rec、INFRA 如何计）？  
5. 有没有本包数据**不支持**「拆机动点」假说的地方？

---

## 8. 相关文件

| 路径 | 说明 |
|---|---|
| `bags/patrol_bc_fast_direct_20261008_150626/results.md` | 本包汇总 |
| `bags/.../run01.log` … `run05.log` | 逐轮全文 |
| `scripts/patrol_bc_fast.sh` | 探针（HEAD 7639970） |
| `scripts/navigate_to_pose_gate.py` | 单点闸门 + recoveries 计数 |
| `ros2_ws/src/inspection_mission/.../patrol_mission_node.py` | 含 V3 航点（本实验未走 split） |
| `ros2_ws/src/navigation_config/config/nav2_test_room.yaml` | Nav2 |
| `docs/咨询-Phase1机动点设计-给外部AI.md` | 几何与 V3 坐标设计 |

---

## 9. 一句话求助

**实验 A 证明：门禁好了，直达 B→C 仍是「常 PASS、高 recovery、偶发无细节 ABORT」。**  
请基于上表给出**唯一下一步**（跑 V3 split / 先加失败分类 / 其他），并说明需要我们再贴什么日志。
