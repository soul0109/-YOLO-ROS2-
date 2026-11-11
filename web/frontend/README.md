# 巡检前端（MVP-R7）

Vue3 + Vite：发起 A→B→C、轮询、逐站结果与归档图。

## 推荐入口（同源 :8000）

```bash
# 历史浏览（无需 Gazebo）
bash ~/inspection-robot/scripts/run_mission_history.sh

# 完整巡检冷启动（Gazebo+Nav2+Runtime+Web）
bash ~/inspection-robot/scripts/run_mission_full.sh
```

默认 DB：`bags/inspection_archive.sqlite3`（不绑定某次证据目录）。  
看 R7 成功归档时显式指定：

```bash
INSPECTION_ARCHIVE_DB=~/inspection-robot/bags/mvp_r7_through_20261110_111120/inspection_archive.sqlite3 \
  bash ~/inspection-robot/scripts/run_mission_history.sh
```

- VM：`http://127.0.0.1:8000/`
- 宿主机：`http://192.168.203.129:8000/`
- 停止：`bash scripts/run_mission_history.sh stop` 或 `run_mission_full.sh stop`

演示步骤与限制见 [`docs/演示说明-MVP巡检.md`](../../docs/演示说明-MVP巡检.md)。

## 约束

- 不直接 `/cmd_vel`、不写 SQLite、不用服务器本地路径做 `<img src>`。
- 图片：`/missions/{id}/stations/{A|B|C}/screenshot|annotated`
- 默认打开根路径，不强制 `?mission_id=`。
