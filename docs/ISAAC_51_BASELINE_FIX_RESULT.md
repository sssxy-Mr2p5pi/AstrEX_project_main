# Isaac Sim 5.1 / Isaac Lab 2.3.2 基线验收结果

STATUS: HISTORICAL
Current baseline: [docs/ISAAC_51_DEV_BASELINE.md](ISAAC_51_DEV_BASELINE.md)
Historical raw evidence was archived to:
`/data/shared/AstrEX_project_data/archives/isaac/2026-09-baseline-consolidation/`
See `MANIFEST.json` for original-path mapping (includes paths kept in place and paths deleted).

验证日期：2026-09-17。测试环境为 `isaaclab232_test`，Isaac Sim `5.1.0.0`，Isaac Lab `v2.3.2`（`37ddf626871758333d6ed89cf64ad702aef127d0`），Python 3.11，Torch `2.7.0+cu128`。系统 ROS 2 是 Jazzy / Python 3.12。DDS 使用 Fast DDS，`ROS_DOMAIN_ID=63`。

本报告中的 `E` 指独立证据目录 `/home/sssxy/Projects/isaac51_fix_Tt8SLc6e`。原始脚本、JSON、日志、截图和 checkpoint 均在 `E`，不在 AstrEX Git 仓库中。本次没有修改官方任务源码或 AstrEX 应用代码。`PASS` 只表示所列条件在本机通过。

## 1. OmniGraph 根因状态

**CONFIRMED ROOT CAUSE：尚未确认单一根因。** 已确认原始 Lab `isaaclab.python.headless.kit` 路径无法创建本轮图；同一机器、同一包、同一 Cartpole，在完整 `isaaclab.python.kit` experience 的 headless 模式下可创建图。单凭这一对照，不能确定是哪项 extension 或设置造成差异。原始错误是 `Failed to wrap graph in node`，不是 DDS 连接错误。证据：`E/original/result.json`、`E/full_headless_B/result.json`。

**SUPPORTED WORKAROUND：**本轮官方 ROS 图使用完整 Lab experience，即使窗口关闭仍用其 headless 模式。图使用 `isaacsim.core.nodes.OnPhysicsStep` 触发，并设置 `GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND`。一环境 JointState 发布与控制使用进程级 CPU PhysX 和 `clone_in_fabric=False`。这些都是测试进程配置；未修改官方 experience 或 Cartpole 源码。证据：`E/ros_control_sim.py`、`E/repeat_analysis.json`。

**UNRESOLVED HYPOTHESIS：**默认 headless experience 的 `omnigraph.updateToUsd=false`、`omnigraph.disablePrimNodes=true`、`useSchemaPrims`、Bridge 启用顺序或其组合可能影响建图。单变量覆盖均未单独修复。后续如要恢复默认 headless 路径，需要再缩小 extension/setting 差异；不要据此改系统 ROS。

另一个独立问题已定位：最初使用 `OnPlaybackTick` 与底层 `sim.step()` 时，订阅命令可到达，但控制图没有随 physics step 执行。改为官方 5.1 的 `OnPhysicsStep` + OnDemand 后，ROS 命令进入 Controller 并产生实际运动。这解释了该控制试验的“有命令、无运动”，不等于解释原始 graph prim 创建失败。

## 2. Bridge extension 加载顺序

`isaacsim.ros2.bridge` 通过 ExtensionManager 启用，并以 application update 等待初始化。图创建前确认 `ROS2PublishClock`、`ROS2PublishJointState`、`ROS2SubscribeJointState`、`ROS2Context` 的 node type 已注册。A–E 在 node type 可用时仍失败，因此“未等待 Bridge 初始化”不能单独解释 headless 建图故障。证据：`E/A/result.json` 至 `E/E/result.json`。正式闭环使用相同 node type，图中没有 direct-rclpy 发布器。

## 3. 图路径冲突

原始路径 `/World/AstrEXValidationGraph` 在建图前不存在。每次诊断改用 `/World/AstrEXROSGraph_<UUID>`，默认 headless 仍失败；完整 experience 中 UUID 路径成功。`GRAPH_PATH_COLLISION_CONFIRMED=false`。不能把路径冲突写成根因。证据：`E/original/result.json`、`E/A/result.json`、`E/full_headless_B/result.json`。

## 4. Stage、edit target 和 timeline

原始复现时 stage/root layer 是同一个匿名 `World0.usd`，session layer 是 `World0-session.usda`。edit target 指向 root layer 且可写。Cartpole 的 `/World` 和 Robot prim 存在，`SimulationContext` 是当前对象，timeline 为 Play；建图路径上没有 prim。完整 experience 的成功组在同类匿名 stage、可写 edit target 和 Play timeline 下创建了 `OmniGraph` prim。因此本轮证据不支持“stage 不存在”或“edit target 不可写”。详细 identifier 与 cache ID 见 `E/original/result.json`、`E/full_headless_B/result.json`；这些每次进程都会不同。

## 5. Graph timing A–E

各组使用独立进程、1 个环境、seed 42；均使用默认 headless experience。Bridge 的四类 ROS node type 已注册。

| 组 | 建图时机 | 结果 | 首个失败点 |
|---|---|---|---|
| A | env 创建前 | FAIL | Graph prim 无法包装成节点 |
| B | env 创建后 | FAIL | Lab 设 seed 时 Replicator `SDGPipeline` 建图失败 |
| C | `env.reset()` 后 | FAIL | 同 B，未到本轮图创建 |
| D | reset + 两次 update 后 | FAIL | 同 B |
| E | timeline Play 后 | FAIL | 同 B |

A 组无法创建本轮图，所以没有“env 创建后是否保留图”的有效观察。将相同 B 时机切换到完整 experience 后 PASS。证据：`E/A`～`E/E` 与 `E/full_headless_B` 的 `result.json`、`run.log`。每个进程均受 600 秒上限约束。

## 6. Headless 设置隔离

默认 headless 启动时实测 `updateToUsd=false`、`disablePrimNodes=true`、`useSchemaPrims=true`。对前两项分别做单变量覆盖，另测试 schema/extension 条件；仍未修复默认 headless 建图。纯 Sim 与完整 Lab experience 可建图。正式测试用独立 `user.config.json` 与关闭持久设置写入的启动参数，避免污染用户配置。早期诊断曾使 Kit 持久配置短暂变化；已恢复并在结束时核对为 `false/true`。证据：`E/override_updateToUsd`、`E/override_disablePrimNodes`、`E/useSchemaPrims_A`、`E/schema_A`、`E/full_headless_B`。完整 experience 是已验证 workaround，不是对单个设置的根因证明。

## 7. 官方 ROS 控制链

官方链：`/joint_command` → `ROS2SubscribeJointState` → `IsaacArticulationController` → CPU PhysX → `ROS2PublishJointState` → `/joint_states`。`OnPhysicsStep` 同时驱动官方 `/clock`。所有控制组不调用 `env.step(action)`；每控制步调用两次 `base.sim.step(render=False)`，再刷新 scene。物理步长 `1/120 s`。没有从 Lab action 管线写入另一组 effort。Cartpole articulation root 为 `/World/envs/env_0/Robot`；joint 为 `slider_to_cart` 和 `cart_to_pole`。

| 门槛 | 结果 | 证据 |
|---|---|---|
| Bridge 与 node type | PASS | `E/repeat_*/sim_result.json` |
| 官方 `/clock` | PASS | 系统端 ≥10 个严格递增时间戳，`E/clock_run/system_result.json` |
| 官方 `/joint_states` | PASS | 系统端 ≥10 条有限、变化的两个关节状态，`E/joint_cpu/system_result.json` |
| 官方 subscriber 与 Controller | PASS | 指令 0、+5、−5 N 与 subscriber/controller 输入相同 |
| ROS→PhysX 运动 | PASS | 下表；比较共同 reset 前第 49 步 |
| PhysX→ROS feedback | PASS | 各组 110～122 条；时间误差 0 或 −1 ns，位置/速度误差约 `3.3e-5` 以下 |
| 新进程重复闭环 | PASS | `E/repeat_zero`、`repeat_positive`、`repeat_negative` |
| 原 effort 回归 | PASS | `E/effort_regression_1/result.json`、`E/effort_regression_16/result.json` |

| 输入 | 相对零输入位置差 | 相对零输入速度差 | 判定 |
|---|---:|---:|---|
| +5 N | +0.37266767 m | +0.20937936 m/s | PASS |
| −5 N | −0.37266767 m | −0.20937936 m/s | PASS |

两个方向均超过 1 mm 和 0.01 m/s，方向正确。命令、subscriber 输出、Controller 输入、位置、速度、reset 与反馈原始序列见 `E/repeat_*/sim_result.json` 和 `system_result.json`；归纳见 `E/repeat_analysis.json`。本控制路径未验证的 Lab actuator effort buffer 不用于 PASS 判定，记作 N/A。

GPU PhysX 的早期 JointState 试验出现空 joint 名与 tensor device mismatch；CPU Physics + `clone_in_fabric=False` 后发布通过。该差异是当前 workaround 的适用范围，不应外推成多环境 GPU ROS 控制已通过。

## 8. direct-rclpy 诊断基线

此前独立验证已证明 Sim 内置 Python 3.11 Jazzy 与系统 Python 3.12 Jazzy 通过 Fast DDS 交换标准消息：`/clock`、`/joint_states`、ROS→Sim 和 Sim→ROS 均 PASS。原始证据在 `/home/sssxy/Projects/isaac51_validation_c30sx014/ros/round2`，前次报告是 `docs/ISAAC_51_LAB232_VALIDATION.md`。`DIRECT_RCLPY_DDS_BASELINE=PASS`，但这些结果不计入本报告的官方 OmniGraph gate。

## 9. GUI 结果

ROS GUI 自动试验打开 X11 `:1` 的 Isaac Sim 窗口，1 个 Cartpole 运行 125 秒。官方图存在，交替发送 +5/−5 N，保存 `E/gui_ros/ros_gui.png`、`sim_result.json` 和 `system_result.json`。自动遥测和窗口/截图存在均 PASS。用户确认场景与 Timeline 正常，未观察到穿透、爆炸或持续冻结；快速自动切换不足以肉眼判断方向。因此另做手动分段检查。

`E/gui_manual_direction_retry2/` 的分段过程为 reset → 0 N 约 2 秒 → +5 N → 人工暂停 → reset → 0 N 约 2 秒 → −5 N 约 3 秒 → 人工暂停。用户分别明确回复正向 PASS、负向 PASS。正向运动在约 2.95 秒时触发杆角越界保护，提前约 0.05 秒暂停；不能记作严格完整的 3 秒保持。正向滑车到 +0.374 m，速度 +0.170 m/s，ROS 反馈一致；负向滑车到 −0.337 m，速度 −0.628 m/s，ROS 反馈一致。第二次 0 N 保持末存在约 +0.012 m 小幅漂移，可能受前一段命令/状态切换影响；未将其写为完全静止。四张截图、逐阶段命令/状态/反馈与用户 PASS 记录均在该目录。

随后重新打开 GUI，选中 `/World/AstrEXROSGraph_87cd9c844ec74351a2d6a6d8a18a5ba6`。用户在 Stage/Action Graph 中实际看到了控制图及 Clock、JointState Publisher/Subscriber 节点，并明确回复 PASS。程序记录在 `E/gui_graph_inspection/sim_result.json`。因此本轮 GUI 的人工条件已齐备；方向判定以浅蓝色滑车为准，不把深蓝色杆子的旋转误判为滑车方向。

RL GUI 使用 100-update checkpoint，1 环境运行 65 秒、2504 控制步、9 次 reset；截图为 `E/rl_eval/rl_gui.png`。用户反馈“目测正常”，看到滑车循环，杆子未明显旋转。截图显示 Cartpole 场景。日志中有一次 Fabric cloner error，但进程正常退出、状态有限；这个 warning 需要保留，不能把用户反馈解释为对该日志的排除。GUI 总 gate 还取决于 ROS GUI 人工结果。

## 10. RSL-RL 安装与依赖变化

官方 v2.3.2 `source/isaaclab_rl/setup.py` 要求 `rsl-rl-lib==3.1.2`、`onnxscript>=0.5`。安装前这两包不存在。`pip --dry-run` 显示只需新增依赖，不要求改变 Torch、Isaac Sim、Isaac Lab、packaging 或 Pillow。执行 `./isaaclab.sh --install rsl_rl`；首次运行在末尾 VS Code 设置的非交互 EULA 提示中退出，但 Python 依赖已经安装。随后设置官方 `OMNI_KIT_ACCEPT_EULA=YES` 重跑，脚本 exit 0。证据：`E/rl_official_dryrun.json`、`E/rl_install.log`、`E/rl_install_accepted.log`。

新增包：`rsl-rl-lib 3.1.2`、`onnxscript 0.7.2`、`onnx-ir 1.0.0`、`tensordict 0.14.2`、`orjson 3.12.0`、`GitPython 3.1.62`、`gitdb 4.0.12`、`smmap 5.0.3`、`importlib_metadata 9.0.1`、`zipp 4.1.0`、`pyvers 0.2.3`。**升级、降级、删除：均无。** 重装的本地 editable Lab 组件版本未变化。`isaaclab_mimic` 对 `rsl_rl` extra 发出“不提供该 extra”的非阻断提示。

安装后 `pip check` 仍报告四组 metadata 冲突：wheel/packaging、fastapi/starlette、isaacsim-kernel/psutil、isaacsim-kernel/typing-extensions。相关已安装版本在安装前后未变化，因此这些是本轮前已有的组合问题，不是新增 RL 依赖导致的版本变化。本任务按保护规则不调整它们。原始输出：`E/pip_check_after.txt`。功能 PASS 也不消除这些依赖声明冲突。

## 11. PPO smoke 与稳定性

使用官方 `scripts/reinforcement_learning/rsl_rl/train.py`、官方 `Isaac-Cartpole-Direct-v0` 配置、16 环境、seed 42、headless。ROS command 控制关闭。训练进程各自独立，未修改官方 PPO 算法。

| 项目 | 20 updates | 100 updates |
|---|---:|---:|
| 进程 | exit 0 | exit 0 |
| actor 最大参数变化 | 0.301985 | 0.704718 |
| critic 最大参数变化 | 0.636068 | 0.964703 |
| checkpoint optimizer steps | 400 | 2000 |
| 有限标量曲线 | PASS | PASS |
| 连续 5 次异常 loss 阈值 | 未触发 | 未触发 |
| mean reward 首→末 | −3.46 → 21.12 | −3.46 → 192.58 |
| mean episode length 首→末 | 25.0 → 53.93 | 25.0 → 204.29 |

100-update value loss 范围 `0.961–188.060`，surrogate loss 范围 `−0.0270–0.0465`，entropy 范围 `1.058–1.388`。所有 TensorBoard 标量有限。训练日志没有 PhysX error 或 native crash；回合正常 reset。20-update 原始日志、TensorBoard 和 checkpoint 在 `E/rl_smoke/`；100-update 在 `E/rl_stability/`。逐 update 曲线与阈值计算见两处 `metrics.json`。短跑不表示策略长期收敛。

## 12. Checkpoint reload 与 600-step evaluation

100-update 训练进程完全退出后，新进程读取 `model_99.pt`。checkpoint SHA256 为 `fea10794570514b7dbefeb16bc33b0bd0a75dd0cfa34b23f2269b383b105605d`，模型参数 SHA256 为 `20dd5b2a599bfc6d8c09d757c6f2ab6514b19c6944eebea368365a9282e38390`。记录了网络参数键和固定零观测的 policy output。16 环境执行 600 步，动作、奖励、位置、速度均有限，位置范围 `−2.856–2.998 m`，33 次 reset，进程 exit 0。`CHECKPOINT_RELOAD=PASS`。证据：`E/rl_eval/result.json`、`eval_retry.log`、`E/rl_evaluate.py`。第一次评估的日志因测试记录脚本将写 JSON 放在 app 关闭之后而缺少结果；调整隔离脚本顺序后新进程复测 PASS，未改官方代码。

## 13. 十分钟稳定性

该测试以 ROS GUI 的人工基本确认作为前置条件。用户已确认场景、Timeline、无明显异常、手动正负方向与控制图/节点可见性。第一轮 `E/ros_stability_10min/` 的 Sim 运行 `600.004 s`，系统端得到 51,002 条递增 Clock 和 51,002 条 JointState，零次 >5 秒停更，无错误；但系统端独立计时只有 `599.367 s`，低于严格 600 秒门槛。该轮**不判 PASS**。

第二轮 `E/ros_stability_10min_retry/` 留出 10 秒余量。Sim 运行 `610.029 s`，系统监听 `609.945 s`，两个进程均 exit 0。系统收到 52,202 条严格递增 Clock 与 52,202 条 JointState；关节名正确、位置和速度均有限。无 >5 秒 topic 停更、无记录到的 DDS/PhysX 错误、无 native crash；边界正常 reset 492 次，命令正负切换 122 次。两条 topic 的最大采样年龄分别约 0.00085 s、0.01081 s。故**十分钟通信与物理功能门槛 PASS**。原始数据见 `sim_result.json`、`system_result.json`，摘要与日志错误扫描见 `analysis.json`。

内存单独标注未解决：前 2 分钟 warm-up 后，RSS 的 50 次每十秒抽样从 `5,182,779,392` 增至 `5,185,720,320` bytes，单调增加 `2,940,928` bytes（约 0.057%）。同段 GPU 已用显存在 4,767–4,813 MiB 波动，首尾为 4,767→4,787 MiB。RSS 增幅很小，但本轮不能解释其来源，也没有证明长期平台期。因此 `MEMORY_TREND=UNRESOLVED`；十分钟功能 PASS 不等于长期内存稳定。

## 14. 安装前后包清单

完整记录：`E/pip_freeze_before.txt`、`E/pip_freeze_before_rl.txt`、`E/pip_freeze_after_rl.txt`、`E/pip_freeze_after.txt`、`E/pip_freeze_final.txt`，以及对应的 `conda_export_before.txt`、`conda_export_after.txt`、`conda_export_final.txt`。RL 安装后的 freeze 与全部测试结束后的 freeze 完全相同。安装前后的差异只有第 10 节列出的新增包，没有已安装包版本变化。Conda export 警告：环境中有大量 pip 安装包，Conda export 不能保证完整复现；这不等于运行时故障。

## 15. 保护检查与最终门槛

- Lab HEAD 前后均为 `37ddf626871758333d6ed89cf64ad702aef127d0`，`git status --short` 前后为空；见 `E/lab_git_before.txt`、`lab_git_after.txt`、`lab_git_final.txt`、`lab_head_final.txt`。
- 官方 Cartpole、PPO 脚本的最终 SHA256 在 `E/official_sha256_final.txt`，与前一次快照 `official_sha256_after.txt` 相同。本次未编辑这些文件；Git 状态支持源码未改动。
- 未修改 Isaac Sim/Lab/Torch/CUDA 版本、系统 ROS、kernel、驱动或 Isaac 6 环境。没有 commit/push。只允许的包变更是第 10 节的 RL 依赖。
- `ASTREX_ROS_CONTROL_BASELINE=NO`（保守判定）。官方闭环与严格十分钟功能测试均 PASS，但 warm-up 后 RSS 持续单调增长、原因未明。按本计划“无法解释的持续增长不宣称稳定”的规则，当前不把该组合宣布为完整稳定 ROS 控制基线。它可用于本机短时开发与定位；长期使用前需要确认内存趋势的来源或平台期。
- `ASTREX_RL_BASELINE=YES`：effort 1/16、20/100 updates、checkpoint reload、600-step evaluation 均 PASS。本机本轮达到该限定门槛；不代表复杂 RTX 场景或长训练稳定。
- `ASTREX_GUI_BASELINE=YES`：用户已确认 ROS GUI 场景/Timeline、正负方向、Action Graph/ROS 节点，以及 RL GUI 策略控制画面。该 YES 仅覆盖本机本轮可视化观察；保留第 9 节的 Fabric cloner error 和正向提前保护暂停记录。

本机可继续使用现有 `isaaclab232_test` 做已通过的物理、短时官方 ROS 控制与 RL 工作，但不得自动把它设为 AstrEX 的完整稳定 ROS 控制基线。自定义 `astrex_interfaces`、多环境 GPU ROS 控制与长期维护均未在本轮验证。Isaac Sim 5.1 已停止维护；本报告只给出本机已测范围。
