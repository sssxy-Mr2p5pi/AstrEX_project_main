# CartPole Step 4：移动与顺序保持验收

日期：2026-09-30。状态：P0→P4 完成，STEP4_GUI_MVP=PASS。

本轮只执行 [Step 4 计划](CARTPOLE_STEP4_GUI_MVP_PLAN.md)。Step 3 B/C 已完成，不重新核对其成绩，也不重跑矩阵。

## 1. 实现范围

统一入口复用原控制节点、StateCache、LQR、5 N 限力和官方 ROS 控制图。节点支持 `hold`、`move`、`move_then_hold` 和无发布器的 `dry-run`。本轮没有新增 ROS Action 或正式 Runner 框架。

移动参考使用五次平滑曲线，最大参考速度 0.05 m/s。Move 在参考到达最终目标后，按四状态容差连续判定 1 秒。顺序演示在 Move 完成后发零，等待新的 S1，再以固定最终目标保持 3 秒。两阶段共用一个发布器。

退出规则保持：限力 ±5 N、摆杆 torque 0、杆角 10°、参考误差 0.25 m、连续限幅 0.5 仿真秒、状态失效 0.2 本地秒。移动目标另按实际轨道范围检查。控制与稳定计时都读取反馈的仿真时间。

## 2. 代码检查与构建

- P0 提交：`7484915`，归档 pulse 工具并保存 [历史索引](CARTPOLE_EXPERIMENT_HISTORY.md)。
- P1 提交：`24ccc21`，统一任务入口与独立分析器。
- P2 提交：`d5f865f`，严格 dry-run 验收。
- P3 提交：`66f7571`，GUI 实测与逐物理步日志修正。
- P1 相关单元测试：131 项通过。测试消息只存在于测试进程；ROS 节点测试使用 domain 97，不向真实 domain 63 注入状态。
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

M1、M2 分别运行于新的 GUI 进程。参考速度保持 0.05 m/s。用户确认 M1 向画面右侧缓慢移动、杆保持平稳；M2 向左移动，视觉正常。

| 工况 | 最终目标 | 实测控制时间 | Move 稳定窗口 | 最大发布力 / Controller 输入 | 结果 |
| --- | ---: | ---: | ---: | ---: | --- |
| M1 | +0.3 m | 12.250000638 s | 1.000000052 s | 0.471359 / 0.471359 N | PASS |
| M2 | −0.3 m | 12.250000639 s | 1.000000052 s | 0.472807 / 0.472807 N | PASS |
| M3，初角 +2° | +0.5 m | 22.758334521 s，含 Hold | 1.000000052 s | 4.317266 / 4.317266 N | DEMO_SUCCESS / PASS |

M1/M2 均无限幅、无 boundary/reset，最终 Controller 输入清零。配置、trace、在线结果和独立 assessment 位于：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/runs/20260930T133835Z_413b38afb6/`。该次总 manifest 保留 FAIL，因为其中 M3 的独立核对未通过。

### 4.1 保留的失败记录

第一次 M1 在线完成移动，但独立核对未通过。GUI 外循环每两物理步记录一次状态，官方 ROS Publisher 每物理步发布。半步反馈不能直接与邻近端点状态比较。原失败位于 `runs/20260930T132124Z_3bd83dcb46/M1/`，没有覆盖。

随后按真实时间对两侧测量做线性重采样。M1/M2 的四状态误差均小于原有 `2e−4` 容差。报告明确区分原始测量与派生值。重采样跨度不超过两个物理步，不做外推或动力学积分。

M3 在线返回 `DEMO_SUCCESS`，但前 0.3 仿真秒中有 17 条半步反馈的速度核对失败。它们处于初角恢复及 effort 快速变化阶段。所有同时间原始测量均匹配，Move→Hold 附近没有此错误。这说明线性速度重采样不足以验收该暂态；在线成功不覆盖独立 FAIL。

最小修正是在官方 post-physics 回调中，逐步读取 articulation 的真实位置和速度，追加 `ros_physics_samples.jsonl`。新日志存在时，分析器要求精确时间匹配，不插值、不回退到稀疏日志。原有 `2e−4` 容差不变。

此修正只增加只读采样。原步进、GUI、K、力上限和成功条件不变。只重跑受影响的 M3，旧记录保持原路径。

### 4.2 M3 最终结果与时序

最终证据：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/runs/20260930T140144Z_b6c811bf87/M3/trial/`。对应 Isaac 原始测量：`/data/shared/AstrEX_project_data/logs/isaac/ros/20260930T140146Z_a1f80d564a/`。

| 时点 | 仿真时间 / 时间戳 | 证据 |
| --- | --- | --- |
| 首条控制反馈 S0 | 0.691666702 s | `[x,x_dot,theta,theta_dot]=[0,0.0015,0.035,0.0067]`。初始化读回是零速度、+2°，S0 已经过一个真实物理步 |
| 参考到达目标后的 Move 稳定窗口 | 19.441667680 → 20.441667732 s | 四项容差连续满足 1.000000052 s |
| Move 成功并停止 Move 更新 | `20441667732 ns` | `x=0.4967 m`；成功之后没有 Move 非零控制输出 |
| 切换零命令发布完成 | 本地 `21613380875451 ns` | WAIT_S1 只发零，不继续计算 Move 控制力 |
| 新 S1 / Hold 开始 | 仿真 `20450001066 ns`，收到本地 `21613381274681 ns` | 晚于 Move 成功一个物理步；晚于发零 0.399230 ms。Hold 目标仍为 +0.5 m |
| Hold 稳定窗口 | 20.450001066 → 23.450001223 s | 独立连续计时 3.000000157 s，没有继承 Move 时间 |
| 整体成功与最终零 | 23.450001223 s | 最终 `x=0.4995 m`，`x_dot=0.0001 m/s`，`theta=0`，`theta_dot=−0.0001 rad/s`；Controller 清零确认 |

2732 条反馈全部匹配真实逐物理步测量，没有使用派生插值。时间表示误差小于 1 ns，四状态最大误差各小于 `5e−5`。原核对容差仍为 `2e−4`。

最大发布力和 Controller 输入均为 4.317266 N，摆杆 torque 为零。连续限幅时间为零，活动阶段没有 boundary/reset。参考速度仍为 0.05 m/s，未采用降速或调 K。

用户对 M3 确认：“看到了，控制期间正常”。人工确认记录与截图位于本轮 `20260930T125010Z_session/`。该确认只覆盖可见演示，数值结论由 trace 和独立 assessment 给出。

### 4.3 当前 hold 入口兼容性回归

统一节点修改了公共控制和连续计时逻辑。按计划补了一个当前 `hold_regression`，初位 0 m、初角 +2°，没有重跑历史 C 矩阵。

结果为 PASS：3.766666863 仿真秒结束，其中连续稳定 3.000000156 s。峰值力与 Controller 输入均为 4.317266 N，无限幅、无 boundary/reset，最终清零。453 条反馈均与逐物理步测量精确配准。

证据位于 `runs/20260930T140355Z_c9d4066e2f/hold_regression/trial/`。本轮当前相关单元测试共 148 项通过。

## 5. 数据与保护边界

配置固定为 Sim 5.1.0.0、Lab v2.3.2、CPU PhysX、domain 63。K、包版本和官方资产未修改。现有四项 metadata warning 保留，没有新增 warning。

保护快照：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/protection/`。执行前后 pip freeze、Conda export、Lab HEAD/状态、受保护文件指纹均逐字一致。官方 Lab 仍为 `37ddf626871758333d6ed89cf64ad702aef127d0`，无源码改动。

用户原有 50 项暂存改动保持原样。暂存差异 SHA256 仍为 `6266494d7b23fcc6f83dc7dbcd459153c3cb7fab53e3711c0f72e9ab1f1b77a9`。没有处理 `ros2_ws/src/tmp` 的原有状态，也没有 push。

Controller 输入不等于 PhysX 实测 applied effort。真实施力字段仍标为 N/A。

## 6. P4 清理与验收

从活动源码移出 10 个退休文件：B/C 两个运行器、两个分析器、四个工具测试，以及旧独立 dry-run 节点与其测试。完整路径见 [历史索引](CARTPOLE_EXPERIMENT_HISTORY.md)。一次性归档位于 `20260930T125010Z_session/history/retired_step3/`；10 个文件的 SHA256 均与 Git `66f7571` 对应内容一致。

7 个退休文件缓存移至 `history/retired_cache/`。旧安装别名 `balance_hold_dry_run`、`balance_hold_closed_loop` 移至 `history/retired_installed_entries/`。它们都可恢复，原始实验数据未移动或删除。

保留当前 `balance_hold_closed_loop_node.py`、Controller、StateCache、参考与计时模块。保留两个 reset/命令隔离专项测试。B/C 报告仅增加 HISTORICAL 说明，正文结论不变。归属无法确认的旧构建日志仍保留。

| 收尾检查 | 结果 |
| --- | --- |
| 当前 `astrex_ros_bridge` 定向重建 | PASS，1.01 s；日志在 `build_log_p4/` |
| 新终端 console entry / metadata / 实际导入 | 仅 `cartpole_trial`、`state_cache`；模块指向当前 build/source |
| 活动源码对退休模块的依赖 | 没有残留导入 |
| 清理后的相关单元测试 | 148 PASS |
| 当前两个工具及 ROS entry 的 `--help` | PASS，不启动仿真 |
| `git diff --check` | PASS |
| 环境、官方源码、K、用户暂存保护 | PASS |

P4 没有改变控制行为，故不重复三组 GUI。所有本轮模拟器和控制进程都已正常关闭。

## 7. 当前使用方式

严格只读计算：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py --mode dry-run
```

GUI 三组试验默认自动顺序执行：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py
```

只看最终顺序演示，并在成功后保留窗口 45 秒：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py --case M3 --keep-gui-seconds 45
```

当前固定目标单例：

```bash
/usr/bin/python3 -B scripts/run_cartpole_trial.py --case hold_regression
```

`--case M1`、`--case M2` 可选择单组。`--wait-before-start` 可让首组等待本次输出目录内的 `gui_continue.json`，内容为 `{"continue": true}`。H1 工具已确认视角，本轮无须人工逐组启动。

失败时停止后续工况。每个模拟器进程上限 600 本地秒。每次创建独立共享输出目录，终端打印真实路径。独立分析器可接收 `trial` 目录；重新分析历史证据时，用 `--output` 指定新的文件，避免覆盖原 assessment。

## 8. 完成范围

本轮证明小范围 GUI 移动与严格顺序保持可运行，保留了两次独立核对失败及修正记录。不把成功后的自由运动记作控制成绩，也不声称长时间或任意初态稳定。

Step 4 已完成。自定义 ROS Action、Grounder 和正式 Runner 尚未实现。本轮不自动进入下一阶段。
