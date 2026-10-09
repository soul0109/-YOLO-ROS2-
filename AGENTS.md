# Agent / 新对话入口

> **新开聊天先读本文件 + `docs/当前进度.md`**，再动手。不必粘贴旧对话全文。

## 阅读顺序（勿跳过）

1. [`docs/当前进度.md`](docs/当前进度.md) — **NOW**：阶段、阻塞、下一步
2. [`docs/后续开发计划与建模攻关方案.md`](docs/后续开发计划与建模攻关方案.md) — **主执行手册**（任务顺序、建模、验收闸门）
3. [`docs/阶段1验收总结与项目交接包.md`](docs/阶段1验收总结与项目交接包.md) — 阶段证据 + 给其他 AI 的交接约束
4. 过程经验：[`docs/开发历程/`](docs/开发历程/README.md)（只追加；勿用活文档代替）
5. 按需：[`项目执行计划.md`](docs/项目执行计划.md) · [`仓库结构说明.md`](docs/仓库结构说明.md) · [`docs/README.md`](docs/README.md)

协作细节：[`docs/协作方式-VM直开.md`](docs/协作方式-VM直开.md)

---

## 开发者画像（Charles）

| 维度 | 说明 |
|------|------|
| **目的** | 作品集级仿真巡检：Gazebo → SLAM/Nav2 → YOLO 异常 → Web；纯仿真交付；LLM 仅任务层 |
| **背景** | Python / FastAPI / OpenCV / YOLO 熟；ROS2/Gazebo 在学；要**能跑、能验收、能写简历** |
| **你要扮演的角色** | 技术导师 + 系统集成搭档：验收闸门、可运行输出，**不做纯课程式科普** |
| **沟通** | 中文；先结论；给可复制命令；他常追问「为什么要有这层 / 真机要不要」——分层回答 |
| **环境** | VMware Ubuntu 22.04，`~/inspection-robot`；WASD 遥控；Git 同步见协作文档 |

**偏好**：最小 diff、不 over-engineer；Web/YOLO 段落少讲基础；ROS/Gazebo/Nav2 段落讲清集成与踩坑。

**雷区**：勿改 24.04/Jazzy 主路径；勿 `sync_from_share`；键盘勿嵌 launch；未验收不跳阶段；勿新建第四份总计划。

---

## 新对话推荐开场（复制给 AI）

```text
先读 AGENTS.md 和 docs/当前进度.md，按执行手册从当前进度接着推进。
验收闸门内持续推进；里程碑结束：更新 docs/当前进度.md（短）+ 追加 docs/开发历程/ 一条。
```

有具体目标可补一句（可选），例如「接着做阶段 5 YOLO」。有报错再补：完整日志 + 命令 + 文件片段 + 已尝试方案。

---

## 硬约束（已定）

- 主环境：**Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11**
- 改进度 → `docs/当前进度.md`（NOW）+ 追加 `docs/开发历程/`（THEN）；计划调整 → `docs/后续开发计划与建模攻关方案.md`
- 主开发在 VM `~/inspection-robot`；`bash scripts/git_sync_push.sh "msg"` 同步
- 每阶段 checklist 全绿再进下一阶段

## Cursor 规则

- `.cursor/rules/project-context.mdc` — 项目读序与硬约束
- `.cursor/rules/developer-profile.mdc` — 开发者画像与协作人格（与上文同步）
