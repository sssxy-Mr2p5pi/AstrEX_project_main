# CartPole Step 4：移动与顺序保持验收

日期：2026-09-30。状态：P0/P1/P2 完成；GUI 闭环待验收。

本轮只执行 [Step 4 计划](CARTPOLE_STEP4_GUI_MVP_PLAN.md)。Step 3 B/C 已完成，不重新核对其成绩，也不重跑矩阵。

## 1. 实现范围

统一入口复用原控制节点、StateCache、LQR、5 N 限力和官方 ROS 控制图。节点支持 `hold`、`move`、`move_then_hold` 和无发布器的 `dry-run`。本轮没有新增 ROS Action 或正式 Runner 框架。

移动参考使用五次平滑曲线，最大参考速度 0.05 m/s。Move 在参考到达最终目标后，按四状态容差连续判定 1 秒。顺序演示在 Move 完成后发零，等待新的 S1，再以固定最终目标保持 3 秒。两阶段共用一个发布器。

退出规则保持：限力 ±5 N、摆杆 torque 0、杆角 10°、参考误差 0.25 m、连续限幅 0.5 仿真秒、状态失效 0.2 本地秒。移动目标另按实际轨道范围检查。控制与稳定计时都读取反馈的仿真时间。

## 2. 代码检查与构建

- P0 提交：`7484915`，归档 pulse 工具并保存 [历史索引](CARTPOLE_EXPERIMENT_HISTORY.md)。
- P1 提交：`24ccc21`，统一任务入口与独立分析器。
- 相关单元测试：131 项通过。测试消息只存在于测试进程；ROS 节点测试使用 domain 97，不向真实 domain 63 注入状态。
- 旧 `astrex_ros_bridge` build/install 已移动至共享历史目录，不删除源码 symlink 指向的内容。
- 仅定向构建 `astrex_ros_bridge`，构建成功。新终端确认实际导入路径和 `ros2 run astrex_ros_bridge cartpole_trial --help`。

旧构建归档和本轮构建日志位于：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/`。

## 3. 严格 dry-run

三个计算共用一个真实 Isaac 只读会话。场景使用 CPU PhysX、domain 63、Fast DDS 和完整 experience。初始化握手未释放，场景保持零状态。

| 工况 | 参考 | 理论参考时长 | 结果 | 实际发布力 / Controller 输入 |
| --- | --- | ---: | --- | --- |
| D1 | 0 → +0.3 m | 11.25 s | DRY_RUN_PASS | 0 / 0 N |
| D2 | 0 → −0.3 m | 11.25 s | DRY_RUN_PASS | 0 / 0 N |
| D3 | 0 → +0.5 m | 18.75 s | DRY_RUN_PASS | 0 / 0 N |

原始 trace、配置、结果、独立 assessment 和 manifest：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/runs/20260930T131145Z_6a6509f888/`。

对应 Isaac samples/events：`/data/shared/AstrEX_project_data/logs/isaac/ros/20260930T131147Z_e0f6ef3eed/`。

三次节点都未创建命令发布器，未写 `trial_ready.json`，没有 command 事件。分析器从保存的 K、四状态和时间戳独立核对参考及建议力。Controller 输入保持零，无 reset/boundary。进程正常退出并确认已停止。

这些结果只证明接线、计算和时间规则。它们不证明移动恢复。`F_raw` 与 `F_cmd` 在 dry-run 中都是建议值，实际没有发送。

## 4. GUI 实测

待按 M1、M2、M3 顺序运行。失败时停止后续工况，保存失败记录。M3 的人工视觉确认尚未取得。

## 5. 数据与保护边界

配置固定为 Sim 5.1.0.0、Lab v2.3.2、CPU PhysX、domain 63。K、包版本和官方资产未修改。现有四项 metadata warning 保留，没有新增 warning。

保护快照：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/protection/`。结束后还需比较包清单、Lab 状态、受保护文件和用户暂存差异指纹。

Controller 输入不等于 PhysX 实测 applied effort。真实施力字段仍标为 N/A。

## 6. 当前使用方式

严格只读计算：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py --mode dry-run
```

GUI 三组试验：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py --wait-before-start
```

首组等待 `gui_continue.json` 观看准备信号。后两组通过后自动进入下一组；失败则停止。每个模拟器进程上限 600 本地秒。全部产物保存在共享数据，不进入 Git。

Step 4 完成后，才移除已替代的阶段专用程序。本报告此时不宣称 GUI 或整个 Step 4 已通过。
