# 协作方式（VM 直开，2026-08-28 起）

> **主开发环境：** VMware Ubuntu 22.04，`~/inspection-robot`  
> **同步到宿主机：** **Git 唯一主路径**（`git push` / `git pull`）

## 日常流程

```bash
cd ~/inspection-robot
bash scripts/run_gazebo_teleop.sh --build    # 仿真 + 方向键
bash scripts/smoke_test_stage33.sh           # 自测刹车链（无需按键）
```

## 同步到宿主机

| 方式 | 何时用 | VM 命令 | 宿主机 |
|------|--------|---------|--------|
| **A GitHub（推荐）** | 网络通 | `bash scripts/git_sync_push.sh "feat: xxx"` | `git pull origin main` |
| **B bundle 离线包** | push 失败 | 同上（失败时生成 `~/inspection-robot-vm.bundle`） | `git pull <bundle路径> main` |

**一键（推荐）：**

```bash
bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述本次改动"
```

- push **成功** → 宿主机 `git pull origin main`
- push **失败** → VM 生成 `~/inspection-robot-vm.bundle`，宿主机用 bundle 拉取后再 push

**宿主机 → VM（偶发，如宿主机写了文档）：** 宿主机 `git push` 后，VM `git pull`；或从 VMware 共享盘 **只复制指定文件**（勿整仓 rsync `--delete`）。

## 每次推进：收工同步（固定习惯）

VM 是唯一主环境。每完成一个小任务（改代码 / 改 world / 勾阶段），按下面做：

```text
改代码 → VM 验收 → 更新 docs/当前进度.md（+ 对应阶段说明）→ git_sync_push.sh "msg"
```

| 改了什么 | 顺手更新 |
|---|---|
| 阶段进度、下一步、踩坑 | `docs/当前进度.md` |
| 世界布局 / 坐标 / 验收 | `docs/阶段X说明.md` |
| 任务顺序 / 验收闸门 | `docs/后续开发计划与建模攻关方案.md` |
| 新增阶段说明 | `docs/README.md` 索引加一行 |

**宿主机接住：**

```bash
git pull origin main
```

## 已删除 / 勿用

- ~~`sync_from_share.sh`~~ **已删除**（曾用 `--delete` 把 VM 新文件删掉）
- ~~`sync_to_share.sh`~~ **已删除**（整仓 rsync 镜像，易误删；统一走 Git）

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
| **B 宿主机推** | VM commit 后 push 失败 → 用 bundle 或宿主机在 clone 里 pull/commit/push |
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
