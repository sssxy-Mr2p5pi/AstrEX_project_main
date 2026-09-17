# Isaac Sim 5.1 / Isaac Lab 2.3.2 隔离验证

STATUS: HISTORICAL
Current baseline: [docs/ISAAC_51_DEV_BASELINE.md](ISAAC_51_DEV_BASELINE.md)
Historical raw evidence was archived to:
`/data/shared/AstrEX_project_data/archives/isaac/2026-09-baseline-consolidation/`
See `MANIFEST.json` for original-path mapping (includes paths kept in place and paths deleted).

验证日期：2026-09-17。结论：**不推荐现在替换 AstrEX 的现有开发基线**。物理与标准消息 DDS 通信通过；Lab Cartpole 场景中的官方 ROS 2 OmniGraph 发布图未能创建。下文把直接使用内置 `rclpy` 得到的结果与官方图节点验收严格分开。

原始证据目录：`/home/sssxy/Projects/isaac51_validation_c30sx014`（下文简称 `E`）。该目录、源码克隆和新 Conda 环境均在 AstrEX 主仓库之外。主仓库只新增本报告。

## 1. Exact Versions

| 项目 | 实测版本或位置 |
|---|---|
| OS | Ubuntu 24.04.4 LTS，x86_64 |
| Python | 新环境 3.11.16；系统 `/usr/bin/python3` 3.12.3 |
| Conda 环境 | `/home/sssxy/miniconda3/envs/isaaclab232_test` |
| Isaac Sim | `isaacsim[all,extscache]` 实际 distribution `5.1.0.0`；不是 5.0/6.x |
| Isaac Lab | 官方 tag `v2.3.2`，commit `37ddf626871758333d6ed89cf64ad702aef127d0`；核心包 metadata `0.54.2`，任务包 `0.11.12` |
| Isaac Lab 源码 | `/home/sssxy/Projects/IsaacLab-2.3.2-test`，最终 Git 状态干净 |
| PyTorch | `torch 2.7.0+cu128`，`torchvision 0.22.0+cu128`，`torchaudio 2.7.0+cu128` |
| CUDA | PyTorch CUDA 12.8，`torch.cuda.is_available() == True` |
| ROS 2 | 系统 Jazzy，Sim 内置 Jazzy；Fast DDS (`rmw_fastrtps_cpp`)，domain 63 |

包快照：`E/freeze.txt`、`E/conda_export.txt`。后者同时包含 Conda 的 stderr 提示，作为导出记录保存，不作为可直接执行的纯 YAML 锁文件。

## 2. Installation Result

**PASS（安装完成，含非阻断依赖警告）。** 新环境使用 Python 3.11。先在官方 cu128 索引安装三件套，再安装完整 Sim 5.1，随后从 tag `v2.3.2` 按 `./isaaclab.sh --install none` 安装 Lab 组件。安装前检查了解析结果；没有使用 `--no-deps`、修改依赖声明或改动目标版本。没有安装本轮不需要的 RL 框架。

初次运行官方安装脚本时，`TERM=dumb` 导致 `tabs` 命令立即失败；设 `TERM=xterm` 后同一安装脚本成功。旧 `flatdict` 的构建预检查还需要新环境内的 `setuptools 80.9.0`。这些处理仅作用于新环境，日志见 `E/lab_install.log` 与 `E/lab_install_retry.log`。

最终 `pip check` 有四项**未解决**的 metadata 不一致，不把安装成功写成“依赖完全无冲突”：`wheel 0.47.0`/`packaging 23.0`，`fastapi 0.115.7`/`starlette 0.49.1`，`isaacsim-kernel 5.1.0.0`/`psutil 7.2.2`，以及 `isaacsim-kernel`/`typing_extensions 4.16.0`。当前功能测试通过不能证明这些组合在其他工作负载中安全。完整版本见 `E/freeze.txt`。

## 3. CUDA Result

**PASS。** `torch`、`torchvision`、`torchaudio` 均能导入。CUDA 可用；一次小型 GPU 矩阵乘法返回预期数值。该检查不是性能测试。

## 4. Isaac Sim Result

**PASS（headless 启动）。** 最小 `SimulationApp` 正常启动，创建 PhysX 场景，推进 60 步并关闭。测得仿真时间约 `0.5166667 s`。证据：`E/sim_smoke.py`、`E/sim_smoke_result.json`、`E/sim_smoke.log`。未测试 GUI、复杂 RTX 场景或长时间运行。

## 5. Isaac Lab Result

**PASS（导入和 Cartpole smoke）。** 官方 `Isaac-Cartpole-Direct-v0` 任务可导入。1 环境和 16 环境分别在第 19 个 action 步前产生有限且变化的 cart 状态，正常退出；两组各环境测得的最大 cart 位移均约 `0.858566 m`。证据：`E/result.json`、`E/smoke16/result.json` 及同目录日志。没有运行 PPO。

## 6. PhysX Effort Result

**PASS，1/1 和 16/16 环境正负方向均通过。** 独立适配脚本 `E/cartpole_validate.py` 调用 Lab 2.3.2 API；没有修改官方任务源码。脚本 SHA256：`ac012a9fbe1f058f40b3f654728d2f4fcd21e1d19feb673e8402ca155e6da1c3`。官方任务文件 `cartpole_env.py` SHA256：`b1e7c79ba0dc6c6ab4c6875b278a3459e660b84849a62d03790dc6828644aef8`。

固定条件：seed 42；`slider_to_cart` 由名称解析为 joint index 0；cart 与 pole 初始位置/速度为 0；`dt=1/120 s`；decimation 2；action scale 100；cart actuator stiffness 0、damping 10；沿用官方任务的越界和 episode reset。每组单独 reset，输入按 `0, +0.5, -0.5` 顺序执行，对应目标 `0, +50, -50 N`。每组最多 60 步，出现任一 reset 即停止该组。三组共同的最后一个 reset 前样本是第 18 步；reset 帧没有参与比较。

| 环境数 | 正方向相对零输入 | 负方向相对零输入 | 判定 |
|---|---|---|---|
| 1 | Δ位置 `+0.858566 m`，Δ速度 `+3.434622 m/s` | Δ位置 `-0.858566 m`，Δ速度 `-3.434622 m/s` | PASS，1/1 |
| 16 | 全 16 个环境与上行数值一致 | 全 16 个环境与上行数值一致 | PASS，16/16 |

第 18 步的 target 为 ±50 N；可读取的 computed/applied effort buffer 均为约 ±15.275539 N。**目标力不等于实际施力缓冲区**，所以判定依据是位置、速度相对零输入的真实响应，而非缓冲区非零。所有判定样本数值有限，方向正确，位移严格超过 1 mm，速度严格超过 0.01 m/s。逐步、逐环境原始记录及 reset 标志见 `E/effort1/effort_trajectories.json`、`E/effort16/effort_trajectories.json`；汇总见各目录 `result.json`。

## 7. Comparison with Isaac 6

历史 A/B 证据位于 `/home/sssxy/Projects/physx_ab_JPThAicl/comparison.json`，共同有效步为 18。两个 Isaac 6 + Lab 3.0 beta2 patch1 组合的 effort 验收都为 FAIL；本轮 5.1 + 2.3.2 的 16 环境结果为 PASS。因此记录为 `LEGACY_STACK_PASS_NEW_STACK_FAIL`。这里同时改变了 Isaac Sim 与 Isaac Lab 的大版本、Torch 及测试实现；**不能单独归因于 Sim 6 的 PhysX 回归**。旧组合的结果不因本轮实验而改写。

## 8. ROS2 Jazzy Architecture

Sim 使用 Python 3.11 与其内置 Jazzy 库；系统节点使用 `/usr/bin/python3` 3.12 与 `/opt/ros/jazzy`。两进程仅通过 DDS 通信，`ROS_DOMAIN_ID=63`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 一致。Sim 启动前清除了继承的 `PYTHONPATH`、`AMENT_PREFIX_PATH` 和系统 ROS 路径；系统侧独立加载 Jazzy。系统 `ros2 doctor --report` 已执行。

`isaacsim.ros2.bridge` 扩展能够加载，Sim 侧 `rclpy` 实际位置为新环境内 `isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy/`；系统侧为 `/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/`。没有把任一 Python 版本的 site-packages 注入另一进程。

**官方发布图验收 FAIL。** 在 Lab Cartpole 场景中，官方 `ROS2PublishClock` 和 `ROS2PublishJointState` 所需的 OmniGraph 图未能创建：日志报 `Unable to create prim for graph at /World/AstrEXValidationGraph`，随后 `Failed to wrap graph in node`。场景与 Robot prim 存在，编辑层显示可写。另一个仅运行纯 Sim 的最小诊断，在相同新环境且正确加载内置 Jazzy 库后，能创建 `ROS2PublishClock` 图（`E/graph_retry/graph_smoke_result.json`）。这将问题定位到**本轮 Lab Cartpole 建图路径**，尚不足以断言其确切根因，也不能把纯 Sim 建图成功算作 Cartpole 官方发布验收成功。

为隔离 DDS 层，在官方图失败后使用 Sim **内置** `rclpy`，从真实 Cartpole 状态构造标准 `Clock`、`JointState` 并发布，且订阅/回传标准 `String`。以下 9–11 节的 PASS 都是这条直接 `rclpy` 路径的结果，不是 OmniGraph 节点的结果。脚本与日志：`E/ros_sim.py`、`E/ros_system.py`、`E/ros/round2/`。

## 9. /clock Test

**PASS（直接 `rclpy` 路径）；官方图路径 FAIL。** 系统端收到 10 个严格递增的 `Clock` 时间戳，首尾分别为 `300000015 ns` 和 `450000023 ns`。证据：`E/ros/round2/ros_system_result.json`。时间来自 Sim 步进，没有用系统墙上时钟冒充仿真时钟。

## 10. /joint_states Test

**PASS（直接 `rclpy` 路径）；官方图路径 FAIL。** 系统端收到 10 条从真实 Cartpole 状态构造的 `JointState`。joint 名称为 `slider_to_cart`、`cart_to_pole`；位置和速度有限且变化。发布器发现及消息原值见 `E/ros/round2/ros_system_result.json`。这不证明官方 `ROS2PublishJointState` 节点可用。

## 11. ROS → Sim Test

**PASS（直接 `rclpy` 路径）。** 系统端发送唯一标识 `astrex51-68d414c3c74748e3add7ad8bdb881ac8` 的 `std_msgs/String`；Sim 侧接收后回传，系统端收到相同标识。系统端也发现本轮的两个节点以及 `/clock`、`/joint_states`、ping、ack topic。证据：`E/ros/round2/ros_sim_result.json` 与 `E/ros/round2/ros_system_result.json`。这证明标准消息 DDS 双向互通，但不证明官方 ROS subscriber 图节点通过。

## 12. Python 3.11 / 3.12 boundary

**PASS（标准消息、直接 DDS）。** Sim 进程报告 Python 3.11.16 和内置 Jazzy `rclpy` 路径；系统进程报告 Python 3.12.3 和系统 Jazzy `rclpy` 路径。两端在 domain 63 完成双向标准消息交互。结论只覆盖本轮标准消息，不代表 Python ABI 可以混用。

## 13. Custom astrex_interfaces compatibility

**CUSTOM INTERFACE PATH: SUPPORTED（官方有支持路径；AstrEX 未实测）。** 正式 `ros2_ws/src/astrex_interfaces` 当前只有包骨架与空 `msg/srv/action` 目录，没有 `.msg/.srv/.action`。本轮未构建、修改或导入该包。未来若 Sim Python 3.11 侧需要自定义类型，应在隔离的 Python 3.11 ROS workspace 中构建；系统 Python 3.12 侧另行构建。两侧使用相同 IDL，但 build/install/log 不能共用。标准 `Clock`、`JointState`、`String` 的 PASS 不能外推为自定义接口 PASS。参见 [Isaac Sim 5.1 ROS 2 安装和自定义接口说明](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)。

## 14. Blocking Issues

1. **BLOCKER：**Lab Cartpole 场景内官方 ROS 2 OmniGraph 发布图创建失败。本轮没有通过官方 `/clock`、`/joint_states` 发布节点验收。纯 Sim 最小图成功，因此先排查 Lab 场景建图时机、stage/edit target 与 OmniGraph 的交互；不要根据现有证据直接改驱动、系统 ROS 或旧环境。
2. **警告：**`pip check` 的四项 metadata 不一致仍在。未遇到这四项引发的核心功能异常，但也没有完成更广工作负载验证。
3. **范围限制：**未做自定义接口、GUI、长 PPO、复杂 RTX、完整 AstrBotEX 接入或长期稳定性测试。Isaac Sim 5.1 的[官方文档](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)已标注该版本停止维护。

## 15. Final Recommendation

**NO。** 5.1/2.3.2 可作为本机已测的隔离 Physics/effort 对照环境，且 Python 3.11 ↔ 3.12 的标准消息 DDS 双向互通。它尚未达到本轮规定的完整 Physics + 官方 ROS 2 Bridge 基线门槛。保留 `isaaclab232_test` 用于后续只针对建图失败的诊断；不自动切换 AstrEX 启动配置，也不替换或删除原有 Isaac 6 环境。即使后续补齐 ROS 验收，也只可称“本机已测基线”，不能声称全面稳定或长期受维护。

### 完整验收矩阵

| 验收项 | 结果 | 证据或限制 |
|---|---|---|
| Isaac Sim startup | PASS | `E/sim_smoke_result.json` |
| Isaac Lab import | PASS | 官方 wrapper 导入检查；源码 tag/commit 已核对 |
| CUDA | PASS | 三件套导入、CUDA True、GPU 小运算 |
| PhysX | PASS | 最小场景与 Cartpole 步进 |
| Cartpole 1 env | PASS | `E/result.json` |
| Cartpole 16 env | PASS | `E/smoke16/result.json` |
| Effort 1 env | PASS | `E/effort1/result.json`，逐步 JSON |
| Effort 16 env | PASS | `E/effort16/result.json`，逐步 JSON |
| ROS 2 Bridge 扩展加载 | PASS | 内置 Jazzy `rclpy` 可加载 |
| 官方 ROS 2 OmniGraph 发布节点 | FAIL | Lab 场景创建 graph prim 失败；纯 Sim 最小图 PASS |
| `/clock` | PASS（直接 `rclpy`）；官方图 FAIL | 系统端 10 个递增样本 |
| `/joint_states` | PASS（直接 `rclpy`）；官方图 FAIL | 系统端 10 条真实、变化的关节状态 |
| ROS topic/node discovery | PASS | 本轮 2 节点、4 个指定 topic 均可见 |
| ROS → Sim | PASS（直接 `rclpy`） | 唯一标识回执匹配 |
| Python 3.12 ↔ 3.11 DDS | PASS（标准消息） | 两端 `rclpy` 路径各自正确，无跨版本注入 |
| AstrEX 自定义接口 | NOT RUN | 仅官方支持路径分析，无接口定义可测 |

### 配置与保护检查

两个原有环境的 `pip freeze` SHA256 在测试前后未变化：`isaaclab60` 为 `a5e65288c7072de18278811eb495639ba2dad06f281267656ba2e4d8e51f5bd7`；`isaaclab60_6010test` 为 `40d3906727a33c3b815fc6db8946135ce60cd93977d80488b6ab80226cc61f1c`。原 `/home/sssxy/Projects/IsaacLab` 仍在 `ffff603eafc6b74264a5261cc0183d6a65390d78`，工作树干净。新克隆工作树也干净。本轮没有修改正式 ROS workspace、Conda 原环境、系统 ROS、驱动或 kernel，没有 commit/push。
