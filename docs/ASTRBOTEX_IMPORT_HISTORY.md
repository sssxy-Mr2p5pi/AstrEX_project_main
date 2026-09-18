# AstrBotEX 导入期记录（合并）

STATUS: HISTORICAL
Historical raw material was archived to:
`/data/shared/AstrEX_project_data/archives/astrex/2026-09-repo-consolidation/`
See `MANIFEST.json` for original-path mapping.

本文件合并了 2026-09-16 的仓库初始化报告与 AstrBot 预 submodule 对照清单，只保留结论、约定与证据位置；
两份原文归档在上述归档根。app 层的现状说明保留在 [`docs/ASTRBOTEX_ARCHITECTURE_REVIEW.md`](ASTRBOTEX_ARCHITECTURE_REVIEW.md)。

## 1. 仓库初始化（2026-09-16）

- **目录骨架**：`apps/`（AstrBot submodule、AstrBotEX 应用、adapters、skill_registry）、`ros2_ws/`、
  `sim/`、`config/`、`scripts/`、`docs/`、`runtime/`；大文件与生成物放到仓库外
  `/data/shared/AstrEX_project_data`。
- **AstrBot 引入方式**：作为 Git submodule，来源 `https://github.com/AstrBotDevs/AstrBot.git`；旧的
  AstrBot 快照移到 `/home/sssxy/Projects/AstrBot-4.28.1-pre-submodule`（保留至今，仅作对照）。
- **忽略与 LFS 约定**：根 `.gitignore` 排除编辑器/agent 状态、Python 与 JS 缓存、密钥与环境文件、
  `ros2_ws/{build,install,log}`、runtime 日志与临时 rosbag、生成 USD、rosbag 与训练权重、以及
  `sim/assets/versioned/` 之外的二进制资源；Git LFS 只跟踪
  `sim/assets/versioned/**/*.{usdc,usdz,onnx}`。文本 `.usda` 与普通 `.usd` 不全局忽略。
- **ROS 2 接口脚手架**：建立 `ros2_ws/src/astrex_interfaces`（ament_cmake 空接口包），首次
  `colcon build`/`colcon test` 通过；当时该包尚无 msg/srv/action 定义。本地 colcon 产物已在
  2026-09 清理（可随时重建）。
- **当时已知告警**：`git lfs status` 把 submodule gitlink 显示为 `?: <missing>`（直接 Git 与
  submodule 检查均为该 commit 的干净状态）；`git diff --cached --check` 报 4 处既有 AstrBotEX 文档
  的行尾空格（未修改这些文件）；仓库当时没有 remote，也没有 commit/push。

## 2. 预 submodule 快照对照（41 项差异）

- **统计**：37 个文件被修改、4 个条目只存在于旧快照、0 个条目只存在于官方 submodule。
- **旧快照独有**（未拷回 submodule）：`astrbot/core/computer/local_file_security.py`、
  `astrbot/core/computer/process_sandbox/`、
  `dashboard/src/components/shared/LocalPermissionMatrix.vue`、`tests/test_local_sandbox_access.py`。
- **处置约定**（此后一直沿用）：不把本地文件拷进 AstrBot submodule；本地运行时适配器放
  `apps/adapters/`，AstrEX 的策略与界面改动放 `apps/AstrBotEX/`。

## 3. 与当前状态的关系

初始化提交 `f2d041f` 与 AstrBotEX 导入提交 `4a199bb` 保留在 Git 历史中；`apps/AstrBotEX` 的代码审阅
（当时基线 `4a199bb`、包版本 0.1.0）仍是当前 app 层文档。Isaac 侧的基线与历史见
[`docs/ISAAC_51_DEV_BASELINE.md`](ISAAC_51_DEV_BASELINE.md) 与 [`docs/ISAAC_HISTORICAL.md`](ISAAC_HISTORICAL.md)。

## 4. 全文与证据位置

| 内容 | 位置 |
|---|---|
| 原初始化报告全文 | `astrex/2026-09-repo-consolidation/astrbtex_import/ASTRBOTEX_INITIALIZATION_REPORT.md` |
| 预 submodule 对照清单原文 | `astrex/2026-09-repo-consolidation/astrbtex_import/astrbot_pre_submodule_diff.txt` |
| 归档根与映射 | `/data/shared/AstrEX_project_data/archives/astrex/2026-09-repo-consolidation/`（`MANIFEST.json`） |
| 旧 AstrBot 快照（保留） | `/home/sssxy/Projects/AstrBot-4.28.1-pre-submodule` |
| app 层架构审阅（当前文档） | [`docs/ASTRBOTEX_ARCHITECTURE_REVIEW.md`](ASTRBOTEX_ARCHITECTURE_REVIEW.md) |
