# 协作方式（VM 直开，2026-08-28 起）

> **主开发环境：** VMware Ubuntu 22.04，`~/inspection-robot`  
> **同步到宿主机：** Git 优先；push 失败则 **VM → 共享文件夹**

## 日常流程

```bash
cd ~/inspection-robot
bash scripts/run_gazebo_teleop.sh --build    # 仿真 + 方向键
bash scripts/smoke_test_stage33.sh           # 自测刹车链（无需按键）
```

## 同步到宿主机（三选一）

| 方式 | 何时用 | VM 命令 | 宿主机 |
|------|--------|---------|--------|
| **A GitHub** | 网络通 | `bash scripts/git_sync_push.sh "feat: xxx"` | `git pull origin main` |
| **B 共享文件夹** | push 失败 | 同上（脚本**自动** `sync_to_share`） | 直接打开共享目录里的项目 |
| **C bundle 离线包** | 没挂共享 | push 失败时生成 `~/inspection-robot-vm.bundle` | `git pull <bundle路径> main` |

**一键（推荐）：**

```bash
bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述本次改动"
```

- push **成功** → 宿主机 `git pull`
- push **失败** → 自动把 VM 代码镜像到共享文件夹 + 生成 bundle

**仅同步共享文件夹（不 commit）：**

```bash
bash ~/inspection-robot/scripts/sync_to_share.sh
```

## 已删除 / 勿用

- ~~`sync_from_share.sh`~~ **已删除**（曾用 `--delete` 把 VM 新文件删掉）

## Cursor / AI

- 在 VM `~/inspection-robot` 直接改代码
- 方向键必须在 **VM 普通终端** 跑 `run_gazebo_teleop.sh`，不能嵌在 launch 子进程里
