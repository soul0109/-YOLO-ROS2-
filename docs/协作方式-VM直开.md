# 协作方式（VM 直开，2026-08-28 起）

> **主开发环境：** VMware Ubuntu 22.04，`~/inspection-robot`  
> **代码同步：** Git 远程（GitHub），不再依赖宿主机共享文件夹

## 日常流程

```bash
cd ~/inspection-robot
# 改代码 → 编译 → 跑仿真
bash scripts/run_gazebo_teleop.sh --build

# 里程碑后：一键提交并推送（推荐）
bash scripts/git_sync_push.sh "feat: 描述本次改动"
```

### 宿主机拉取（替代共享文件夹 sync）

**方式 A：GitHub（推荐，VM 能 push 时）**

```bash
# VM 先推送
bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述改动"

# 宿主机拉取
cd <你的仓库目录>
git pull origin main
```

**方式 B：VM push 失败时，用 bundle 文件（不删任何东西）**

```bash
# VM 生成增量包（在 VM 执行）
cd ~/inspection-robot
git bundle create ~/inspection-robot-vm.bundle origin/main..HEAD

# 把 ~/inspection-robot-vm.bundle 拷到宿主机（U 盘 / 共享只读复制均可）
# 宿主机仓库里执行：
git pull /path/to/inspection-robot-vm.bundle main
```

**不要用 `sync_from_share.sh` 覆盖 VM；也不要用 rsync --delete 双向同步。**

## 自动化自测（无需按键）

```bash
bash ~/inspection-robot/scripts/smoke_test_stage33.sh
```

通过后再做方向键人工验收。

## 已废弃（勿用）

| 脚本 | 说明 |
|------|------|
| `sync_from_share.sh` | 宿主机共享 → VM，曾用 `--delete` 删 VM 新文件 |
| `sync_to_share.sh` | VM → 共享，宿主机开发时代遗留 |
| `sync_pull.sh` / `sync_push.ps1` | 可被 `git pull` / `git push` 替代 |

若误跑 `sync_from_share.sh`，用 `git checkout -- .` 或 `git pull` 恢复。

## Cursor / AI

- 直接在 VM 仓库里改 `~/inspection-robot`
- **不要**在改代码后跑 `sync_from_share.sh`
- 每个新终端：`source /opt/ros/humble/setup.bash && source ~/inspection-robot/ros2_ws/install/setup.bash`
