# 2026-11-09 MVP-R4 SQLite 归档

- **状态：** 待确认（已实跑 + 跨重启 CLI；**不宣称 90% / 正式 Test PASS**）
- **阶段：** MVP-R4
- **证据：** `bags/mvp_r4_20261109_161253/`（mission `mvp_r4_a_001`；`inspection_archive.sqlite3`）
- **相关：** `mission_archive.py` / `mission_archive_cli.py`；R3 同帧视觉结果入档

## 当时目标

Runtime 单写 SQLite：任务终态 + 站点 JSON 路径可持久化；重启后可只读查询；重复 mission_id 拒收。

## 现象 / 阻塞

- 实跑前需清残留图 + AMCL active + DISPLAY；与 R3 同类环境问题，非归档逻辑。
- 重启 Runtime 时 `-p sqlite_db:/path` 会被 rcl 误解析（路径里的 `:`）；必须用 `-p sqlite_db:=/path`。

## 试过什么（含否决）

- 新增 `MissionArchive`（missions/stations）+ `mission_archive_query` CLI
- Runtime：`sqlite_db` 参数、启动 seed、终态 `_archive_after_terminal`；写失败改 FAILED/`archive_failed`
- 离线单测 + 真实 A 站写入：COMPLETED + inspection FAIL（合同预期，非视觉成绩）
- 停 Runtime → 同 DB 重启 `seeded_missions=1` → CLI 查出原记录 → 重复 `mvp_r4_a_001` **Goal was rejected**
- **否决：** FastAPI/Vue/B-C/正式 Test；改 world/路线；commit/push；宣称 90%

## 结论

R4 最小归档链路成立：Runtime 写、重启可读、ID 去重跨进程。等待简短审查。

## 下次别踩

ROS2 覆盖参数一律 `name:=value`；含冒号的路径用 `:=` 否则节点起不来。
