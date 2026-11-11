# 演示说明 · MVP 巡检（R1–R7 开发闭环）

> 开发闭环已完成；**正式视觉 Test / 最终交付验证未完成**。不宣称 90%。

## 配置（Web / API / Runtime 共用）

| 项 | 默认 | 环境变量 |
|---|---|---|
| SQLite 归档 | `bags/inspection_archive.sqlite3` | `INSPECTION_ARCHIVE_DB` |
| 截图输出 | `bags/mission_captures/` | `INSPECTION_CAPTURE_DIR` |
| HTTP 端口 | `8000` | `PORT` |
| 默认路线 | `test_room_ABC` | （前端 / API schema） |
| 出生位（world） | `(0.9, 3.0, yaw=0)` | `SPAWN_WORLD_*` |

集中定义：`scripts/mission_defaults.sh`。  
**默认入口不绑定**某次 `bags/mvp_r7_*` 证据目录或历史 `mission_id`。

## 两个入口

### 1）历史浏览（无需 Gazebo）

```bash
cd ~/inspection-robot
# 默认空库也可启动；要看 R7 成功三站归档时显式指定（不覆盖该证据）：
INSPECTION_ARCHIVE_DB=~/inspection-robot/bags/mvp_r7_through_20261110_111120/inspection_archive.sqlite3 \
  bash scripts/run_mission_history.sh
```

- VM：http://127.0.0.1:8000/  
- 宿主机：http://192.168.203.129:8000/  
- 停止：`bash scripts/run_mission_history.sh stop`

打开根路径（**不要**默认带 `mission_id`），在「最近归档」点选任务。

### 2）完整巡检（冷启动）

```bash
cd ~/inspection-robot
bash scripts/run_mission_full.sh
# 停止：
bash scripts/run_mission_full.sh stop
```

流程：冻结 `nav2_test_room.launch.py` → 确认 GT 在出生位附近 → **只发一次** initialpose → `wait_for_nav_ready` → Runtime + Web。  
**机器人移动后不要再发默认充电位 initialpose。**

## 页面怎么演示

1. **发起三站**：点「发起 A→B→C 巡检」（`route_id=test_room_ABC`）。仅 HTTP 202 且 `accepted` 后算已接受。
2. **看阶段**：当前任务区轮询 `phase` / `status`（ACCEPTED → RUNNING → COMPLETED|FAILED|CANCELED）。
3. **切站与图**：站点 Tab 切换 A/B/C；「原图 / 标注图」切换；图走 API，不用服务器本地路径塞进 `<img>`。
4. **执行失败 vs 巡检异常**  
   - **执行态** `status`：导航/采集/推理是否跑完（如 `FAILED` + `nav_failed`）。  
   - **巡检** `inspection_status` 与各站对象：视觉判定（PASS/FAIL/UNKNOWN）。  
   - 汇总规则：**FAIL > UNKNOWN > PASS**；UNKNOWN **不得**掩盖已确认 FAIL。  
   - 可以：执行 COMPLETED 且巡检 FAIL；或执行 FAILED 但已完成站仍有巡检结果。
5. **查历史**：右侧「最近归档」；无 Runtime 也可读库。
6. **看错误日志**  
   - Web：`bags/mission_web_uvicorn.log`  
   - Runtime：`bags/mission_runtime.log`  
   - Launch：`bags/mission_nav2_launch.log`  
   - 完整巡检入口打印的路径为准。

## 关键证据（保留，勿覆盖）

| 说明 | 路径 |
|---|---|
| 旧逐 via 失败（`B_to_C_via0`） | `bags/mvp_r7_20261109_172354/` · `mvp_r7_abc_20261109_172616_web` |
| ThroughPoses 成功三站 | `bags/mvp_r7_through_20261110_111120/` · `mvp_r7_abc_through_20261110_111332` |

## 已知限制（演示时主动说）

- Nav2 recovery / 偶发拒收：脏图或 lifecycle 未就绪时可能 `nav_goal_rejected`；需干净冷启动，不靠改冻结参数。
- 取景余量：站位与相机高度固定，边缘框可能紧；未做自动对位重试。
- 视觉为**开发模型**（`train1_dev`），非正式 Test 冻结权重。
- **正式视觉 Test / 最终交付验证未完成**；不宣称 90%。
- 取消路径以单测为主；完整入口未强制演示取消。
- B→C 为一次 `NavigateThroughPoses(vias+C)`；via 不是巡检站。
