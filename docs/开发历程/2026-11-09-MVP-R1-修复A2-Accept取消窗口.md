# 2026-11-09 MVP-R1 修复 A2：ACCEPT→Context 取消窗口

- **状态：** 待确认（补丁已实施；**不宣称 R1 PASS**）
- **阶段：** MVP-R1
- **相关：** 修复 A 复审 FAIL — `_goal_callback` ACCEPT 后、`_handle_accepted` 前 `_cancel_callback` 因无 Context 误 REJECT

## 当时目标

只修 Action 接收/Context 安装取消竞态；不进 R2；不修缺陷 2。

## 试过什么

- `AcceptedGoalWindow` + `AcceptCancelBridge`：ACCEPT 时 open ticket；cancel 按 mission_id/generation/goal_handle 记 `pending_cancel`
- `handle_accepted` → `install_context_after_accepted` 消费 pending
- 测试：`TestActionServerAcceptCancelRace` 覆盖 callback 交错（非 Runner send 交错）

## 结论

窗口竞态已补；R1 整体仍 FAIL，待复审。未 commit/push。
