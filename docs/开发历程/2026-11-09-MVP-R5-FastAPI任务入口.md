# 2026-11-09 MVP-R5 FastAPI 任务入口

- **状态：** 待确认（已实跑 HTTP A 站；**不宣称 90% / 正式 Test PASS**）
- **阶段：** MVP-R5
- **证据：** `bags/mvp_r5_20261109_163152/`（mission `mvp_r5_a_001`）；历史冒烟 `bags/mvp_r5_20261109_hist/`
- **相关：** `web/backend/mission_api/`；`MissionArchiveReader`；R4 Runtime 单写

## 当时目标

同 VM FastAPI：提交任务、查运行态、查 SQLite 历史与归档图；Runtime 仍是唯一执行者/写库者。

## 现象 / 阻塞

- 无仿真即可证明 R4 历史查询与图片接口。
- 干净会话 HTTP POST 真实 A 站：202 接受后 poll 到 RUNNING→COMPLETED；COMPLETED+FAIL 合法。

## 试过什么（含否决）

- 拆 `MissionArchiveReader`（`file:?mode=ro` + `PRAGMA query_only`）；CLI 不再经 writer 建表
- 单 Node / 单 executor 线程 ActionClient；uvicorn `--workers 1`
- API：POST/GET missions、stations、screenshot/annotated（路径只来自 DB）
- 离线单测 11 项；历史查 `mvp_r4_a_001`；实跑 `mvp_r5_a_001`
- **否决：** Vue；B/C；正式 Test；改冻结导航/world/机器人/路线；commit/push；宣称 90%

## 结论

HTTP→ExecuteInspection→Runtime→SQLite→HTTP 最小闭环成立。等待简短审查。

## 下次别踩

API 与 Runtime 必须同 DDS / 已 source install；`INSPECTION_ARCHIVE_DB` 与 Runtime `sqlite_db` 指向同一文件。
