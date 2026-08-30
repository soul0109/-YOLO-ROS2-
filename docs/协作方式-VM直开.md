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
| **B 共享文件夹** | push 成功/失败 | 同上（成功**也** `sync_to_share`；失败另生成 bundle） | 直接打开共享目录里的项目 |
| **C bundle 离线包** | 没挂共享 | push 失败时生成 `~/inspection-robot-vm.bundle` | `git pull <bundle路径> main` |

**一键（推荐）：**

```bash
bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述本次改动"
```

- push **成功** → 宿主机 `git pull`；若已挂载 VMware 共享，脚本会**顺带** `sync_to_share`
- push **失败** → 自动把 VM 代码镜像到共享文件夹 + 生成 bundle

## 每次推进：收工同步（固定习惯）

VM 是唯一主环境。每完成一个小任务（改代码 / 改 world / 勾阶段），按下面做，宿主机和共享目录才能跟上。

```text
改代码 → VM 验收 → 更新 docs/当前进度.md（+ 对应阶段说明）→ git_sync_push.sh "msg"
```

| 改了什么 | 顺手更新 |
|---|---|
| 阶段进度、下一步、踩坑 | `docs/当前进度.md` |
| 世界布局 / 坐标 / 验收 | `docs/阶段X说明.md` |
| 任务顺序 / 验收闸门 | `docs/后续开发计划与建模攻关方案.md` |
| 新增阶段说明 | `docs/README.md` 索引加一行 |

**VM 一条命令（commit + push + 刷共享）：**

```bash
bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 简短描述"
```

**宿主机接住：**

```bash
git pull origin main
# 若 Cursor 开的是 VMware 共享文件夹而非 git clone，pull 后仍看不到时：
# 在 VM 再执行 bash scripts/sync_to_share.sh
```

**仅刷共享文件夹（已 commit、不 push）：**

```bash
bash ~/inspection-robot/scripts/sync_to_share.sh
```

共享目录（脚本自动探测）：`/mnt/a_xm/基于YOLO+ROS2的智能巡检机器人仿真系统` 或 `/mnt/hgfs/a_xm/...`

**勿用：** ~~`sync_from_share.sh`~~（单向 VM → 宿主机，反向会删 VM 新文件）

## 已删除 / 勿用

- ~~`sync_from_share.sh`~~ **已删除**（曾用 `--delete` 把 VM 新文件删掉）

## GitHub push 失败排查

**现象（VM 内）：**

```text
kex_exchange_identification: Connection closed by remote host
Connection closed by 198.18.0.159 port 22
fatal: Could not read from remote repository.
```

**根因：** 宿主机科学上网（Clash 等）的 **fake-ip** 把 `github.com` 解析成 `198.18.x.x`。HTTPS 往往还能用，但 **SSH 22 端口**会在密钥交换阶段被断开。不是仓库权限或 SSH key 的问题。

**确认：**

```bash
getent hosts github.com    # 若显示 198.18.x.x 即中招
ssh -T git@github.com      # 22 端口失败
ssh -T -p 443 git@ssh.github.com   # 443 通常可用
```

**修法（任选其一）：**

| 方案 | 操作 |
|------|------|
| **A SSH 走 443（推荐）** | 在 VM `~/.ssh/config` 写入下方片段，再 `git push` |
| **B 宿主机推** | VM `bash scripts/git_sync_push.sh "msg"` → push 失败会自动 `sync_to_share` → 宿主机 `git pull` 或打开共享目录 push |
| **C 关代理 / 改 DNS** | 宿主机 Clash 给 `github.com` 加 `DIRECT` 或 fake-ip-filter；或 VM DNS 改 8.8.8.8（TUN 模式下可能仍被劫持） |

`~/.ssh/config` 片段：

```sshconfig
Host github.com
  Hostname ssh.github.com
  Port 443
  User git
```

## Cursor / AI

- 在 VM `~/inspection-robot` 直接改代码
- 键盘必须在 **VM 普通终端** 跑 `run_gazebo_teleop.sh`（WASD），不能嵌在 launch 子进程里
