# 2026-11-09 MVP-R3 同帧 YOLO+ROI

- **状态：** 待确认（已实跑；**不宣称 90% / 正式 Test PASS**）
- **阶段：** MVP-R3
- **证据：** `bags/mvp_r3_20261109_155952/`（成功 mission `mvp_r3_a_003`）
- **相关：** Hook-1 `scripts/hook1_yolo_roi_e1.py`；M1-T01 UNKNOWN 合同；R2 同帧截图

## 当时目标

ExecuteInspection → A 导航 → 停稳新帧 → **同一截图** YOLO+ROI → 站点结果 JSON → 返回任务结果。

## 现象 / 阻塞

- 首两轮因残留 `cmd_vel_timeout`/未激活 AMCL → `nav_goal_rejected`；清理图后成功。
- 非视觉逻辑问题。

## 试过什么（含否决）

- 新增 `station_inspect.py`：加载已存 PNG → Hook-1 `run_yolo`/`decisions_from_yolo` → `*_station_result.json` + 标注图
- Action Result 增加 `inspection_status` / `station_result_path` / `model_version`
- 灯槽：图像左→右 + `face_neg_y` 映射到设备 leds[]；缺测 UNKNOWN
- 实跑 COMPLETED + inspection FAIL（合同预期）
- **否决：** 改 world/地图/机器人/路线/Nav2；SQLite/Web/B-C；正式 Test；用 GT 当视觉结果；commit/push；宣称 90%

## 结论

R3 最小纵向链路成立：任务执行成功与站点巡检 FAIL 可并存。等待简短审查。

## 下次别踩

启动前确认 AMCL `active` 且 `/initialpose` 有订阅；残留节点会导致拒收导航目标。
