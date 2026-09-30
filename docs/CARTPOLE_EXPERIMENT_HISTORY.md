# CartPole 实验历史索引

日期：2026-09-30。该索引保留证据，不重新验收已经完成的 Step 3。

## 当前入口

当前工作按 [Step 4 计划](CARTPOLE_STEP4_GUI_MVP_PLAN.md)执行。统一入口正在接入移动参考、严格 dry-run 与 GUI 顺序演示。Step 4 实测完成前，不把历史 BalanceHold 成绩写成 MoveCart 成绩。

## 已完成阶段

| 阶段 | 报告与代码版本 | 原始证据位置 | 状态 |
| --- | --- | --- | --- |
| 方向、显式清零与小脉冲 | [Step 3C.5 报告](CARTPOLE_STEP3C5_PULSE_DEBUG.md)，工具归档见下表 | 报告中的 `cartpole_step3c5_*` 与 `ros/*` 原路径 | HISTORICAL；不再默认执行 |
| 时间戳状态缓存 | `8be7aa8` | 源码版本保留于 Git | Step 4 继续复用 |
| BalanceHold B | [B 报告](CARTPOLE_STEP3D_BALANCE_HOLD_B_RESULT.md)，`104f78b` / `9c51837` | `/data/shared/AstrEX_project_data/logs/isaac/balance_hold_step3d/matrix_runs/20260930T094000Z_5ff85d697e/manifest.json`；接通检查另见原报告 | 已完成；不重跑矩阵 |
| BalanceHold C | [C 报告](CARTPOLE_STEP3D_BALANCE_HOLD_C_RESULT.md)，`3ed7aa4` | `/data/shared/AstrEX_project_data/logs/isaac/balance_hold_step3d/c_runs/20260930T114644Z_28cd4e0ee4/manifest.json` | 已完成；Step 4 复用控制链 |

所有原始 trace、Isaac samples/events、manifest、assessment 和结果保持原路径。历史 PASS 只覆盖各自阶段的实验。

## 未跟踪工具的一次性归档

归档根目录：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/history/pulse/`。

| 原活动路径 | 归档文件 | SHA256 |
| --- | --- | --- |
| `scripts/cartpole_pulse_test.py` | `cartpole_pulse_test.py` | `932b097a7e6d03e829f20024210c48c5c71baeab8195c0a065424b5daf6a4b86` |
| `scripts/cartpole_pulse_analysis.py` | `cartpole_pulse_analysis.py` | `fe87ce5b7444d26a1a9a82c2aa606d3972088c61313dfaee872df6b73b9b55ce` |

这两个文件用移动操作归档，未丢弃内容。历史报告已改为指向归档文件。活动源码不再提供小脉冲入口。

## 尚待 Step 4 通过后处理的入口

`run_balance_hold_matrix.py`、`run_balance_hold_c.py`、两个阶段分析器以及旧独立 dry-run 节点仍保留。统一入口完成并通过后，再移除已被接替的代码与专用测试。

`tests/isaac/ros_system_regression.py` 与 `validate_ros_cartpole_boundary_reset.py` 保留为历史专项测试。它们仍覆盖 reset 和命令隔离，本轮不默认执行，也不直接删除。

## 保护快照

本轮保护证据位于 `/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/protection/`。快照包含目标环境包清单、Conda export、Lab HEAD/状态、受保护文件指纹及用户暂存差异指纹。

当前磁盘仅存在 `isaaclab232_test`，没有发现两个旧 Isaac 6 环境。该情况在本轮开始前已存在；本轮不创建、修改或删除环境。
