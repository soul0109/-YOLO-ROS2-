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
