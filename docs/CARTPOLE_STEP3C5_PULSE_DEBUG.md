# CartPole Step 3C.5：开环小扰动不对称排查

> HISTORICAL（2026-09-30）：方向与小脉冲诊断已完成，不再作为当前开发入口。历史结论和原始数据保持有效。本轮不重新运行这些实验。工具已归档，路径和 SHA256 见 [历史索引](CARTPOLE_EXPERIMENT_HISTORY.md)。当前工作按 [Step 4 计划](CARTPOLE_STEP4_GUI_MVP_PLAN.md)执行。

日期：2026-09-29。结论仅适用于 Isaac Sim 5.1.0.0、Isaac Lab v2.3.2、1 个 Cartpole、ROS 官方 OmniGraph 控制链、CPU PhysX、headless 新试验。原有 GUI 日志另作历史对照。本轮没有运行 BalanceHold 闭环，没有修改 Q/R/K、官方资产、Conda 包或系统 ROS。

## 结论

旧的 +0.01 N / −0.01 N 操作**不是同条件实验**：正向从四状态全零开始，在 Controller 输入端持续约 22.367 仿真秒；负向从 `[x=+0.000536464 m, x_dot=0, theta=+0.002181767 rad, theta_dot=0]` 开始，输入约 1.783 仿真秒，随后杆角越界 reset。两次之间没有回到共同零初态。旧日志没有发送端的时间记录，不能证明命令是否由 `--once` 产生；但输入端的非零值持续到显式零命令或 reset，说明不能用人工命令间隔定义“短脉冲”。

重新从独立进程的精确零初态测试 ±0.01、±0.02、±0.05 N。每次按 50 Hz 发送 0.2 秒，随后发送 10 次零力。六次均在 Isaac Controller 输入端看到正确符号、23 个连续非零物理步（约 0.191667 仿真秒）、紧随其后的零输入。三组正负轨迹在共同的前 **84 个物理样本（0.7 秒）**中，四状态镜像误差在记录精度内**全部为 0**。同一时间点的正向响应与力幅值成 1:2:5 比例。故未观察到小扰动方向反转或固定数值死区，也没有证据需要修改线性模型的输入符号。

±0.01 N 在约 0.792 秒后首次出现正负单步差别：负向的两项速度先变为精确 0，正向在该步仍为非零，下一步也归零。这与 PhysX 休眠/阈值效应相符，**具体机制尚未确认**；不能把它解释为有意义的持续受力非对称。诊断用途优先选 **±0.02 N**：短窗口中镜像及比例关系清楚，且比 ±0.05 N 更不易迅速接近越界。这不等于推荐生产控制力。

## 条件与数据边界

- 新试验全部使用 `scripts/start_isaac_ros.sh --headless`，seed 42，1 环境，`dt=1/120 s`，相同 Cartpole 配置、joint `slider_to_cart` 与 `cart_to_pole`；ROS domain 63、Fast DDS。每个符号独立启动一个 Isaac 进程，以新场景初态全零开始。未修改 Isaac 正式运行脚本。
- 脉冲工具：[cartpole_pulse_test.py](/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/history/pulse/cartpole_pulse_test.py)。每次先检查对应 live PID/graph、最近一次 ready 后没有非零 Controller 输入或越界、5 条新的 `/joint_states` 接近零。direct/no-daemon graph 检查要求 `/joint_command` 有 **0 个其他 publisher、1 个属于本轮 Isaac 图的 JointState subscriber**；Fast DDS 隐去节点名时，用 GID 与 direct CLI 查询再次对应。随后发布 `['slider_to_cart','cart_to_pole']` 和 `[±F,0]`，再重复发布 `[0,0]`。
- 发送端 JSONL 记录每条消息的 wall/monotonic 时间；Isaac `ros_samples.jsonl` 逐物理步记录 `subscriber_effort`、`controller_effort`、位置、速度、仿真时间、reset 和 boundary。**`controller_effort` 是官方 Articulation Controller 的输入，不是 PhysX 实测 applied effort**；该字段不能单独证明最终施力大小。状态轨迹证实有实际物理运动。
- `lqr_suggested_force` 仅由离线分析器按当前 K 和目标 x=0 计算。它从未作为本轮命令发布，不能写作实际受力。BalanceHold dry-run 节点也没有 `/joint_command` publisher。
- 现有运行 profile 没有 reset service。每个符号使用独立新进程取得相同初态；用户已授权只重启这一 Isaac ROS 测试进程，未停止其他程序。有效试验全部在首次边界前取得数据。±0.02 N 的正向进程在本比较窗口结束后继续运行并发生杆角越界；该后续事件没有混入镜像分析。±0.01 N 第一次发布工具异常后，仅发出一条非零消息；Isaac 日志没有非零 Controller 输入，已从有效数据中排除，随后使用新的 `plus_001_v2_commands.jsonl` 完成重试。

## 六次有效试验：初态、命令与时长

四状态顺序为 `[x(m), x_dot(m/s), theta(rad), theta_dot(rad/s)]`。六次首个 Controller 非零输入**前一物理步**的四状态均精确为 `[0,0,0,0]`，reset_count 均为 0；各次预检的 5 条实时 JointState 也都是全零。每次力发布 10 条、零发布 10 条，目标频率 50 Hz；表中“发送间隔”是首条力至首条零的实测 wall/monotonic 时间，“接受时长”是 Isaac 中连续同值 Controller 输入的物理步数乘以 `dt`。两种时间不可互相替代。

| 命令 | 首条力→首条零 | Isaac 非零输入步 | 接受时长 | 后一输入 | 运行目录 |
|---|---:|---:|---:|---|---|
| +0.01 N | 0.200110 s | 18962–18984，23 步 | 0.191667 s | 0 N | `20260928T171406Z_38e9c824ff` |
| −0.01 N | 0.200090 s | 2172–2194，23 步 | 0.191667 s | 0 N | `20260928T171816Z_e2a7d03727` |
| +0.02 N | 0.200075 s | 1914–1936，23 步 | 0.191667 s | 0 N | `20260928T172045Z_f186bff666` |
| −0.02 N | 0.200139 s | 3956–3978，23 步 | 0.191667 s | 0 N | `20260928T172231Z_dfad972e8c` |
| +0.05 N | 0.200113 s | 13753–13775，23 步 | 0.191667 s | 0 N | `20260928T172402Z_7c21a7b001` |
| −0.05 N | 0.200115 s | 4249–4271，23 步 | 0.191667 s | 0 N | `20260928T172635Z_6608e39dc9` |

共同前 0.7 秒无 reset、无 boundary。六次 Controller 输入与发送端符号一致；脉冲后第一条记录均为零。只有发布端和输入端可见，具体是哪一层在两条消息之间保持最后值（订阅节点、图或 Controller）不能由这些日志唯一确定。可确定：旧试验的输入没有自动按“短脉冲”结束；本轮显式发零后输入立即归零。

## 高频响应与镜像比较

[分析脚本](/data/shared/AstrEX_project_data/logs/isaac/cartpole_step4/20260930T125010Z_session/history/pulse/cartpole_pulse_analysis.py) 对齐“Isaac 首个非零 Controller 输入步”，比较共同的 reset 前样本。定义四项误差 `mirror_error_y(t)=|y_+(t)+y_-(t)|`，其中 y 依次为 x、x_dot、theta、theta_dot。

| 幅值 | 前 84 步四项最大镜像误差 | 整组共同无越界窗口 | 整组四项最大误差 |
|---|---|---:|---|
| ±0.01 N | 均 0 | 264 步 | x 7.85 µm；x_dot 0.000889 m/s；theta 70.4 µrad；theta_dot 0.00812 rad/s |
| ±0.02 N | 均 0 | 136 步 | 均 0 |
| ±0.05 N | 均 0 | 137 步 | 均 0 |

下面取首个受力样本起第 60 个物理样本（约 0.5 秒）；表中是正向值，负向在这四项上精确取相反数。完整逐步曲线见末尾 CSV。

| 力 | x (m) | x_dot (m/s) | theta (rad) | theta_dot (rad/s) |
|---|---:|---:|---:|---:|
| +0.01 N | +0.0002299083 | +0.0003459337 | +0.0006083662 | +0.0022857084 |
| +0.02 N | +0.0004598165 | +0.0006918668 | +0.0012167323 | +0.0045714159 |
| +0.05 N | +0.0011495402 | +0.0017296571 | +0.0030418325 | +0.0114285303 |

0.02 行是 0.01 行的 2 倍，0.05 行是 5 倍（在所示精度内）。因此 0.01 N **有真实、可分辨响应**，并无严格的“小于 0.02 N 就不动”的死区。自由倒立摆后续会失稳；响应越大，并不表示越稳定。

±0.01 N 在第 95 样本（约 0.792 秒）首次分离：负向 x_dot、theta_dot 同时精确变 0，而正向分别为 +0.000888831 m/s 和 +0.008116958 rad/s；此前 x、theta 仍精确镜像。下一步正向速度也变 0，此后两侧停在略不同位置/角度。当前 cart damping=10，在 0.01 N 的小速度量级与输入同量级；官方模型设置 `sleep_threshold=0.005` 与 `stabilization_threshold=0.001`。这些事实支持“休眠/数值阈值导致离散停动”的假设，但未记录 PhysX sleep flag、接触力或真实 applied effort，故**不能确认具体触发点或将其写成确定根因**。5 Hz dry-run 打印容易漏掉第 95 步前的镜像瞬态。

## 历史试验为何看起来不对称

历史 GUI 数据：[原始逐步日志](/data/shared/AstrEX_project_data/logs/isaac/ros/20260928T151513Z_6091f07f5d/ros_samples.jsonl)、[旧分析摘要](/data/shared/AstrEX_project_data/logs/isaac/cartpole_step3c5_20260929_historical_v2/summary.json)。历史 +0.01 N 从全零初态开始，在 Controller 输入中维持 1342 步、约 22.367 仿真秒；−0.01 N 开始前已有 x=+0.000536464 m、theta=+0.002181767 rad，Controller 仅持续 107 步、约 1.783 仿真秒，随后 pole angle 越界 reset。后一实验没有在 reset 前记录到显式零输入。负向初始正角偏差对小力的影响足够大，使“方向相反必然快速镜像恢复”的判断不成立。旧曲线算出的巨大镜像误差不能用来证明 PhysX 非对称。旧试验发布端是否使用 `ros2 topic pub --once`、准确发布频率和 wall-clock 间隔未留证，仍为未知项。

本轮新试验用**同初态、同接收时长和两端零命令证据**消除了上述混杂因素。新试验为 headless，而旧数据为 GUI；因此不把 GUI/headless 差异单独当作已排除因素，也不声称修复了所有仿真模式下的行为。

## 原因与下一步判定

1. **旧“非对称”主因已确认：**初态不同、输入保持时间不同，负向还触发 reset。旧日志的低频观察加重了错误直觉。
2. **命令到达：**六次 direct graph 检查均满足 0 个额外 publisher、1 个 Isaac subscriber；消息 joint 名称和 `[±F,0]` 正确；Isaac Subscriber/Controller 输入与之匹配。没有发现其他命令源。此检查只对当次启动前瞬间有效，不证明历史任何时刻没有其他节点。
3. **命令保持：**旧输入持续到零/边界；新工具每次显式发零，下一物理步恢复 0。具体锁存层未确认，但不影响“必须显式清零”的操作结论。
4. **越界与 reset：**有效比较窗口中均无；旧负向有。±0.02 N 正向进程在窗口外后续越界，不能把 reset 零帧混入响应。
5. **阻尼/阈值：**0.01 N 有可测响应，低速阻尼可能重要；约 0.79 秒的一步休眠差异是未解决的细机制。当前不能据此声称 PhysX 施力失败或存在固定数值死区。
6. **模型与 LQR：**正力使 x、theta、两项速度为正；负力精确反向，和现有 A/B 输入符号一致。**不需要修正模型符号**。LQR 本轮只是 observer/calculator，**不修改 Q/R/K**，也不宣称该 K 能稳定真实对象。
7. **Step 3D：**小扰动符号排查已经完成，可以据此另行设计受限闭环试验；**本阶段不启动闭环**。进入前应明确力限、人工停机和零命令路径，并单独验证低力下的离散停动不会破坏控制。±0.02 N 适合下一轮开环诊断，不是闭环参数。

## 证据索引与改动范围

共享数据根目录：`/data/shared/AstrEX_project_data/logs/isaac/cartpole_step3c5_live_20260929/`。每个 `pair_001_analysis`、`pair_002_analysis`、`pair_005_analysis` 均有 `summary.json`、`plus_timeseries.csv`、`minus_timeseries.csv`。同级 `plus_001_v2_commands.jsonl`、`minus_001_commands.jsonl`、`plus_002_commands.jsonl`、`minus_002_commands.jsonl`、`plus_005_commands.jsonl`、`minus_005_commands.jsonl` 是逐消息发送时间；各表所列运行目录有 `ros_events.jsonl` 和 `ros_samples.jsonl`。这些都在 Git 仓库外。旧历史数据单独保留，未覆盖。

本任务仅新增/更新本报告及 `scripts/cartpole_pulse_test.py`、`scripts/cartpole_pulse_analysis.py`。未修改原 BalanceHold dry-run、Isaac ROS profile、官方 Cartpole、依赖或其他用户文件；未暂存、commit、push。最终审计：`git diff --check`、`git diff --cached --check` 均无输出，新增三文件也无行尾空格；两个 Python 工具均通过 AST 语法检查。停止目标进程后，没有本轮 `run_ros_cartpole.py` 进程遗留。`git status --short --` 对本轮三个文件均显示 `??`，未暂存。普通 `git diff --stat` 只显示用户原有 `ros2_ws/src/tmp/` 的 8 个删除文件、127 行删除；这是当前工作区的真实结果，未跟踪的本轮文件不会显示其中。原工作区另有用户既有的暂存和修改，未触碰。
