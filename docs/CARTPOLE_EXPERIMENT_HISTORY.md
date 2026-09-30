# CartPole 实验历史索引

日期：2026-09-30。该索引保留证据，不重新验收已经完成的 Step 3。

## 当前入口

当前入口为 `scripts/run_cartpole_trial.py`。严格 dry-run、M1/M2 移动和 M3 顺序保持均通过。结果见 [Step 4 GUI 报告](CARTPOLE_STEP4_GUI_RESULT.md)。历史 BalanceHold 成绩仍只覆盖原有工况。

## 已完成阶段

| 阶段 | 报告与代码版本 | 原始证据位置 | 状态 |
| --- | --- | --- | --- |
| 方向、显式清零与小脉冲 | [Step 3C.5 报告](CARTPOLE_STEP3C5_PULSE_DEBUG.md)，工具归档见下表 | 报告中的 `cartpole_step3c5_*` 与 `ros/*` 原路径 | HISTORICAL；不再默认执行 |
| 时间戳状态缓存 | `8be7aa8` | 源码版本保留于 Git | Step 4 继续复用 |
| BalanceHold B | [B 报告](CARTPOLE_STEP3D_BALANCE_HOLD_B_RESULT.md)，`104f78b` / `9c51837` | `/data/shared/AstrEX_project_data/logs/isaac/balance_hold_step3d/matrix_runs/20260930T094000Z_5ff85d697e/manifest.json`；接通检查另见原报告 | HISTORICAL；不重跑矩阵 |
| BalanceHold C | [C 报告](CARTPOLE_STEP3D_BALANCE_HOLD_C_RESULT.md)，`3ed7aa4` | `/data/shared/AstrEX_project_data/logs/isaac/balance_hold_step3d/c_runs/20260930T114644Z_28cd4e0ee4/manifest.json` | HISTORICAL；Step 4 复用控制链 |
| Step 4 | [GUI 报告](CARTPOLE_STEP4_GUI_RESULT.md)，P0 `7484915`、P1 `24ccc21`、P2 `d5f865f`、P3 `66f7571` | `cartpole_step4/runs/20260930T131145Z_6a6509f888/`、`20260930T133835Z_413b38afb6/` 中 M1/M2、`20260930T140144Z_b6c811bf87/` 中 M3 | 当前；DRY_RUN_PASS 与实控 PASS 分开记录 |

所有原始 trace、Isaac samples/events、manifest、assessment 和结果保持原路径。历史 PASS 只覆盖各自阶段的实验。

## 未跟踪工具的一次性归档

归档根目录：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/history/pulse/`。

| 原活动路径 | 归档文件 | SHA256 |
| --- | --- | --- |
| `scripts/cartpole_pulse_test.py` | `cartpole_pulse_test.py` | `932b097a7e6d03e829f20024210c48c5c71baeab8195c0a065424b5daf6a4b86` |
| `scripts/cartpole_pulse_analysis.py` | `cartpole_pulse_analysis.py` | `fe87ce5b7444d26a1a9a82c2aa606d3972088c61313dfaee872df6b73b9b55ce` |

这两个文件用移动操作归档，未丢弃内容。历史报告已改为指向归档文件。活动源码不再提供小脉冲入口。

## Step 4 完成后的入口清理

以下 10 个文件已移出活动源码，原相对路径保存在共享历史目录 `20260930T125010Z_session/history/retired_step3/`。该路径位于本轮 `cartpole_step4/` 下。

| 退休内容 | 原相对路径 | 当前替代 |
| --- | --- | --- |
| B/C 运行器 | `scripts/run_balance_hold_matrix.py`、`scripts/run_balance_hold_c.py` | `scripts/run_cartpole_trial.py` |
| B/C 分析器 | `scripts/analyze_balance_hold_trial.py`、`scripts/analyze_balance_hold_c_trial.py` | `scripts/analyze_cartpole_trial.py` |
| 四个工具测试 | `tests/test_run_balance_hold_matrix.py`、`tests/test_run_balance_hold_c.py`、`tests/test_analyze_balance_hold_trial.py`、`tests/test_analyze_balance_hold_c_trial.py` | 两个当前 CartPole 工具测试 |
| 独立 dry-run 节点 | `ros2_ws/src/astrex_ros_bridge/astrex_ros_bridge/balance_hold_dry_run_node.py` | 统一节点的 `--dry-run` |
| 该节点专用测试 | `ros2_ws/src/astrex_ros_bridge/test/test_balance_hold_dry_run_node.py` | 当前统一节点与参考测试 |

这些文件还可从 P4 之前的 Git 提交 `66f7571` 恢复。一次性归档不是第二套活动实现。7 个退休脚本缓存及 2 个旧安装别名也已归档。

`setup.py` 仅保留 `state_cache`、`cartpole_trial` 两个入口。原位演进的 `balance_hold_closed_loop_node.py` 保留，当前 Runner 仍依赖该模块。

`tests/isaac/ros_system_regression.py` 与 `validate_ros_cartpole_boundary_reset.py` 保留为历史专项测试。它们仍覆盖 reset 和命令隔离，本轮不默认执行，也不直接删除。

没有清空 `ros2_ws/log`。其中构建日志无法仅凭日期确认归属，故保持原样。本轮构建日志保存在共享会话的 `build_log/` 和 `build_log_p4/`。

## 保护快照

本轮保护证据位于 `/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/protection/`。快照包含目标环境包清单、Conda export、Lab HEAD/状态、受保护文件指纹及用户暂存差异指纹。

当前磁盘仅存在 `isaaclab232_test`，没有发现两个旧 Isaac 6 环境。该情况在本轮开始前已存在；本轮不创建、修改或删除环境。
