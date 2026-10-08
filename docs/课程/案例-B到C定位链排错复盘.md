# 案例复盘：B→C 定位链排错（v32 系列）

> **性质：** 学习向案例，不是验收记录、不是执行手册。  
> **目的：** 把「卡一天却很值钱」的排错过程固化下来，复盘思路、决策树、禁止项与证据路径。  
> **活进度仍以** [`../当前进度.md`](../当前进度.md) **为准**；本文件偏「为什么这么排」，进度文件偏「下一步干什么」。  
> **配套教材：** [`阶段4-导航五课复习.md`](阶段4-导航五课复习.md) 第 0 节（信任分层）+ 第 5 课（排障方法论）

**时间窗：** 2026-10-08（`BC_SPLIT_V3_2` → `V3_3`，探针 `patrol_bc_fast` split）  
**版本标注：** 写到 **v32_6**；后续轮次可在文末「续篇」补一行，不必整篇重写。

---

## 0. 一句话摘要（面试 / 给别人讲）

> B→C 探针表面上挂在 `C_approach` 闸门，真正病灶是 **房内 `+π` 纯 Spin 后 AMCL 发散**。  
> 我们没用「改点位 / 放宽闸门 / 拧 YAML」蒙混，而是：**分层归因 → 最小实验排除假病灶 → 段后定位门禁钉到首发散点 → 再单变量改掉头方式**。

---

## 1. 为什么值得单独记一笔

| 常见错觉 | 这次实际发生的事 |
|---|---|
| 「导航 FAIL = 不会调 Nav2」 | Nav2 常报 SUCCEEDED；GT 闸门不过 |
| 「按理不难，一天也改不好」 | 一天主要花在**排除假病灶 + 缩到首发散点**，不是改不动一个旋钮 |
| 「旧 4.5b 绿过，现在又挂 = 倒退」 | 探针路径更虐（门口两次大转 + 走廊），边界被推高后病灶才暴露 |
| 「赶紧改 alpha / 坐标」 | 那时改会污染归因；病灶位置未定时改参是赌博 |

**作品集价值：** 能讲清「症状 ≠ 病灶」「分类 ≠ 修好」「终点 FAIL ≠ 首发散点」。

---

## 2. 问题背景（当时卡在哪）

### 2.1 探针设定（≠ 4.5b 验收）

- 脚本：`scripts/patrol_bc_fast.sh`（mode=`split`）
- 航点版本：先 `BC_SPLIT_V3_2`，后房内掉头改为 `V3_3`（`B_face_arc`）
- 闸门：`scripts/navigate_to_pose_gate.py`（到位 / 航向 / 停稳；后加 dual-sample）
- 纪律：**本探针 PASS 不能宣布 4.5b**

V3_2 链（示意）：

```text
spawn@B → initialpose → AMCL_LOCKED
→ B_clear (nav)
→ B_face_door (+π 纯 Spin)
→ B_corridor_in (nav，穿 B 门)
→ B_corridor_turn (+π/2 Spin)
→ C_approach (nav) → C
```

### 2.2 表面现象（容易误导的那一层）

早期有效轮常停在：

```text
… → SEG:C_approach:FAIL   # GT pos ≈ 0.25 m > 0.20 m
```

同时常见：`recoveries=0`、Nav2 Action **SUCCEEDED**。  
→ 若只盯「C 点附近」，会误以为要改 `C_approach` 坐标或放宽闸门。

### 2.3 验收尺子 vs 决策尺子（第 0 讲核心）

```text
odom（体感）→ AMCL（对地图认路）→ Nav2（信 AMCL 开车）→ 闸门脚本（信 GT 判分）
```

| 视角 | 数据 | 本案例典型结论 |
|---|---|---|
| Nav2 | AMCL pose + costmap | 到点 SUCCEEDED |
| 闸门 | `/ground_truth` | Position FAIL |
| 诊断 | AMCL↔GT、cov、particle spread | 定位漂了 |

**两件事可以同时成立：** 导航栈「以为到了」，仿真真身还差一截。

---

## 3. 排错总思路（可背的骨架）

### 3.1 分层归因表

航点 FAIL 时先归层，再动手：

| 层 | 问什么 | 本案例用的证据 |
|---|---|---|
| 执行 / 握手 | Action 是否真正跑起来 | Spin accept timeout、retry |
| 规划 / 控制 | 是否 recovery 风暴、无轨迹 | `rec=`、`failed to create plan`、DWB 日志 |
| 闸门语义 | 车到了但取样时机错 | result_time vs settled_time |
| 任务几何 | 点写错、门封死 | 前段 PASS + 几何核对 |
| 定位 | 「我在哪」漂了 | AMCL↔GT、cov、spread；段后 `LOC:` |

**纪律：** 同一时刻只怀疑一层；一次只做一个可证伪假设。

### 3.2 每刀四步循环

```text
1. 写清现象（哪一段、什么码、是否截断）
2. 只提一个假设（「若是 X，应看到 Y」）
3. 最小实验（1 轮 split / 加一种诊断 / 改一处）
4. 记录：排除了什么 / 钉死了什么 → 写入「禁止项」防回退
```

### 3.3 从「哪一层」缩到「哪一步」

```text
终点症状（C_approach FAIL）
    ↓ 分类（dual-sample + AMCL↔GT）
定位层（AMCL 漂移）
    ↓ 段后 LOC 门禁 + 失败即停
首发散点（after_B_face_door）
    ↓ 单变量只改该动作
修法候选（弧线 / 受控转向 …）
```

**关键句：** 分类完成 ≠ 修好；钉到首发散点之后，才谈修法。

---

## 4. 时间线（按轮次，假病灶 → 真病灶）

证据目录惯例：`bags/nav2_diag/patrol_bc_fast_split_v32_N/`

### v32_1 — 认出闸门假 FAIL

| | |
|---|---|
| 现象 | `B_clear` 位姿过，停稳因 `ω≈0.0508` 贴阈值失败 |
| 假设 | 边界噪声，不是主病灶 |
| 结果 | 不当成「定位/规划坏了」；未测到后续 Spin |
| 学到 | 先确认 FAIL 字面是不是真病 |

### v32_2 — 病灶在握手层

| | |
|---|---|
| 现象 | B_clear / B_face_door / B_corridor_in PASS；`B_corridor_turn` Spin **Timeout** |
| 证据 | launch：`Failed to send goal response (timeout)`；behavior 未打印 Turning |
| 归因 | **非几何/DWB**；`/spin` accept 未回到 client |
| 下一刀 | 只动 Spin 握手（重试 / clear 后 dwell），不改 YAML |
| 学到 | 同一条链上失败层级会变；修好的层要从怀疑列表划掉 |

### v32_3 / v32_4 — 钉死「定位漂移」这一层

| | |
|---|---|
| 现象 | 双 Spin `retry=0`；穿门 PASS；挂在 `C_approach` |
| 仪器 | dual-sample：result_time ≈ settled_time |
| 数字（v32_4 settled） | GT pos **0.247 m** FAIL；yaw 6.4° PASS；AMCL↔GT **≈0.29 m**；spread **≈1.13 m** |
| 分类 | `classify_hint`：优先怀疑 **AMCL 漂移** |
| 排除 | 取样过早；点位错（证据不支持）；规划风暴（rec=0） |
| 学到 | Nav2 SUCCEEDED + GT FAIL = 两把尺子；病灶在定位层 |

闸门脚本里的粗分类逻辑（人工仍需确认）：

```text
settled 过、result 不过     → 取样时机
AMCL↔GT 大 / spread 升      → AMCL 漂移
AMCL≈GT 但 GT 仍超闸       → 局部控制或目标姿态
否则                       → 对照 bag 再分
```

### v32_5 — 钉死「首发散点」（最值钱的一刀）

| | |
|---|---|
| 做法 | 每段后 `LOC:`（cov/spread）；诊断 FAIL 即截断，不污染后面归因 |
| 结果 | `B_locked` PASS → `after_B_clear` PASS → **`after_B_face_door` FAIL** |
| 归因 | **房内 `+π` 纯 Spin 本身**触发 AMCL 丢失匹配；不是 C_approach 几何，也还没跑到走廊 Spin |
| 决策 | 下一刀**只替换** `B_face_door`；勿再加 Spin accept 重试 |
| 学到 | 终点症状 ≠ 首发散点；缩到「哪一步」比「哪一层」更可执行 |

诊断阈值（当时）：`cov_xy>0.25` / `cov_yaw>0.20` / `spread>0.5`（诊断门禁，≠ 航点验收闸门）。

### v32_6 — 修法试探：单点弧目标先撞规划墙

| | |
|---|---|
| 改动 | V3_3：房内纯 Spin → 单点 `B_face_arc`（相对 B_clear 仅 Δx=0.30 + yaw 180°） |
| 现象 | `B_face_arc` **NavigateToPose ABORTED**（`failed to create plan` / DWB 无有效轨迹） |
| 结果 | **未测到**「弧线后 AMCL 是否更好」——动作没跑成 |
| 判断 | 单点「短平移 + 大 yaw」又接近「同点换向」坏任务；需拆成**多点、每段带位移**的真弧线再验 |
| 学到 | 修定位假说前，先保证动作在规划层可执行；ABORT ≠ 否定弧线假说 |

---

## 5. 已排除 / 已钉死 / 仍开放

### 已排除（勿回退拧这些）

| 项 | 轮次 |
|---|---|
| 闸门取样过早 | v32_4 |
| Spin accept 握手是主病灶 | v32_3/4 已过；勿再加重视重试当主线 |
| C_approach 坐标写错 | 误差形态像定位；v32_5 首发散在更早 |
| 「只改 YAML / 放宽 0.20·10°」当主解 | 全程纪律 |

### 已钉死

| 项 | 结论 |
|---|---|
| 失败主层 | AMCL 定位发散（不是终点规划挂了） |
| 首发散点 | 房内 `B_face_door` 纯 Spin 之后（v32_5） |

### 仍开放（写本文时）

| 项 | 状态 |
|---|---|
| 「弧线掉头优于纯 Spin」对 AMCL | **未公正验证**（v32_6 动作 ABORT） |
| 多点短弧后 LOC 是否 PASS | 待做 |
| 若多点弧仍爆 | 再谈激光匹配 / IMU·EKF；仍勿拧 C_approach |

---

## 6. 禁止项（方法护栏，不是教条）

写进进度里反复念，是为了防止焦虑时回退：

1. **不改** `C_approach` 坐标「碰运气」  
2. **不放宽** 0.20 m / 10° 冒充修好  
3. **不改** alpha / inflation / Progress / 速度（多变量污染）——除非新证据指向该层且走受控解冻  
4. **不用**每段硬 `/initialpose` 掩盖发散（治标会挡住学因果）  
5. **不 ×10** 糊弄；**不宣布 4.5b**（探针 ≠ 验收）  
6. 握手层已排除后，**勿把「再加 Spin 重试」当定位解**

---

## 7. 工具与证据（以后复盘从哪翻）

| 路径 / 工具 | 作用 |
|---|---|
| `scripts/patrol_bc_fast.sh` | B→C 可解释探针；失败即停；后加段后 `LOC:` |
| `scripts/navigate_to_pose_gate.py` | 三指标闸门 + dual-sample + `classify_hint` |
| `scripts/amcl_health_snapshot.py` | amcl / GT / cov / spread / map→odom |
| `scripts/record_nav2_diag_bag.sh` | 诊断 bag（含 `/particle_cloud` 等） |
| `bags/nav2_diag/patrol_bc_fast_split_v32_*` | 各轮 `results.md` / `runNN.log` / 段日志 |
| [`../当前进度.md`](../当前进度.md) | 当前轮结论与下一刀 |

---

## 8. 和教材第 5 课的对照

| 第 5 课教训 | 本案例落点 |
|---|---|
| 闸门驱动 | 始终以 GT 0.20/10° 判航点；诊断门禁用另一套阈值 |
| 一次一变量 | v32 每轮只动握手 / 只加采样 / 只加 LOC / 只改掉头 |
| 测到声称的能力 | 探针 ≠ 旧 4.5b；弧线假说必须「动作 SUCCEEDED」才谈 AMCL |
| 先仪器化再修 | dual-sample → classify → 段后 LOC → 再改任务路径 |
| 冻结与受控解冻 | YAML 未当主旋钮；改任务几何前写清假设 |

旧 4.5b 事故链是「progress → recovery Spin → 粒子炸 → AMCL 进墙」；  
本案例是「**任务层显式纯 Spin** → 粒子炸」，放大器从 BT recovery 换成了航点设计——**同一类定位脆弱，不同触发器**。

---

## 9. 面试口述（60 秒版）

> 我们在 B→C 链式探针上遇到过「Nav2 SUCCEEDED、GT 闸门 FAIL」。先分层：排除 Spin 握手超时、排除取样过早、排除盲目改终点坐标。用双时刻采样和 AMCL↔GT、粒子 spread 把失败归到定位层，再在每段后做定位健康门禁，发现**第一个发散点是房内 180° 纯 Spin 之后**，而不是走廊终点。所以下一刀只改掉头方式（改成可规划的短弧），不改 Nav2 YAML、不放宽闸门。单点弧曾因规划 ABORT 没测到定位；要拆成多点带位移的弧再验证。这套流程强调症状与病灶分离、单变量、仪器化。

---

## 10. 自测（复盘用）

1. 为什么 Nav2 `SUCCEEDED` 和闸门 `Position FAIL` 可以同时成立？  
2. v32_4 已判「AMCL 漂移」后，为什么还要做 v32_5 段后 LOC，而不是立刻改 alpha？  
3. 若 `after_B_clear` 就 LOC FAIL，决策树相对「首发散在 Spin」应怎么变？  
4. v32_6 的 ABORT 能否否定「弧线优于纯 Spin」？为什么？  
5. 列出三条你下次排障时会主动写进「禁止项」的东西。

（答案不必写在本文；对着第 3～5 节口述即可。）

---

## 11. 续篇（后续轮次只追加）

| 轮次 | 日期 | 一句话结论 | 证据目录 |
|---|---|---|---|
| v32_5 | 2026-10-08 | 首发散点 = 房内纯 Spin 后 | `.../patrol_bc_fast_split_v32_5/` |
| v32_6 | 2026-10-08 | 单点弧 ABORT，弧线假说未测到 | `.../patrol_bc_fast_split_v32_6/` |
| （下一轮） | | | |

---

## 相关文档

- 活进度：[`../当前进度.md`](../当前进度.md)  
- 导航五课：[`阶段4-导航五课复习.md`](阶段4-导航五课复习.md)  
- Nav2 说明（含旧 4.5b）：[`../阶段4.5-Nav2说明.md`](../阶段4.5-Nav2说明.md)  
- AMCL：[`../阶段4.4-AMCL说明.md`](../阶段4.4-AMCL说明.md)  
- 术语外号：[`ROS导航术语外号表.md`](ROS导航术语外号表.md)
