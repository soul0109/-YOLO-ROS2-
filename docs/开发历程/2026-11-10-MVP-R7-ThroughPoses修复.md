# 2026-11-10 MVP-R7 ThroughPoses 修复

- **状态：** 待确认（短审）
- **阶段：** MVP-R7
- **证据：**
  - 失败保留：`bags/mvp_r7_20261109_172354/`（逐 via `NavigateToPose` → `B_to_C_via0` status=6）
  - 修复实跑：`bags/mvp_r7_through_20261110_111120/` · `mvp_r7_abc_through_20261110_111332`
- **相关：** 未改冻结 Nav2/world/地图/模型/坐标；未 commit/push；不宣称 90%

## 当时目标

只修复导航接入语义：B→C 恢复旧 patrol 的一次 `NavigateThroughPoses(vias+C)`。

## 现象 / 阻塞

审查：旧 `patrol_mission_node.navigate()` 在有 via 时一次提交 via+目标；R7 Runtime 拆成独立 `NavigateToPose`，在 via0 失败。

## 试过什么（含否决）

- 最小改 `NavAdapter` + `MissionRunner` + Runtime 调度；A/B 仍 ToPose。
- 接线单测：完整有序 pose 列表、同 stamp、末项为 C。
- **否决**：调坐标/参数/反复清场凑成功。
- 一次干净网页实跑成功，无需失败诊断包。

## 结论 / 产物

| 腿 | Action | 结果 |
|---|---|---|
| →A | NavigateToPose | 到站 + 截图/视觉 FAIL |
| →B | NavigateToPose | 到站 + 截图/视觉 FAIL |
| B→C | NavigateThroughPoses（3 via + C） | 到站 + 截图/视觉 FAIL |

- 任务执行：`COMPLETED`；巡检汇总：`FAIL`
- 三站图像互异（MAE A-B≈12 / A-C≈11 / B-C≈21）；C 用 `face_pos_y`
- 离线测试：R7 route/wiring + R1 runtime **OK**

## 未覆盖

- 本回合未再实机点测取消（仍靠既有单测）
- 不做正式 Test / 90% / commit
