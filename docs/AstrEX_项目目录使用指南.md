# AstrEX 项目目录使用指南（初学者版）

> 目标：让你在开发 AstrEX 时，看到一个新文件就知道“它应该放哪里”，并且知道哪些目录是代码、哪些目录是运行时文件、哪些目录是长期归档数据。

---

## 1. 先记住最重要的一句话

AstrEX 现在分成两个根目录：

```text
/home/sssxy/Projects/AstrEX_project_main/     # Linux ext4：代码与运行区
/data/shared/AstrEX_project_data/             # NTFS：Windows/Linux 共享数据区
```

你可以这样记：

```text
main = 我正在开发和运行的东西
data = 我要保存、共享和归档的东西
```

---

# 2. 总体目录结构

## 2.1 Linux 运行区

```text
/home/sssxy/Projects/AstrEX_project_main/
│
├── apps/
│   ├── AstrBot/
│   ├── AstrBotEX/
│   ├── skill_registry/
│   └── adapters/
│
├── ros2_ws/
│   └── src/
│
├── sim/
│   ├── scripts/
│   ├── config/
│   ├── assets/
│   │   ├── source/
│   │   └── versioned/
│   └── usd_work/
│
├── config/
├── scripts/
├── docs/
├── tests/
├── tools/
│
└── runtime/
    ├── logs/
    └── rosbag_tmp/
```

---

## 2.2 Windows / Linux 共享数据区

```text
/data/shared/AstrEX_project_data/
│
├── documents/
│
├── models/
│   ├── checkpoints/
│   ├── pretrained/
│   ├── exported/
│   └── legacy/
│
├── datasets/
│   ├── raw/
│   ├── processed/
│   ├── annotations/
│   └── external/
│
├── logs/
│   ├── app/
│   ├── isaac/
│   └── rosbag/
│
└── exports/
```

---

# 3. 为什么要分成 ext4 和 NTFS 两个区域？

AstrEX 使用三类存储边界：

- 普通 Git 管理源码、小型文本配置、ROS interface、URDF、Xacro 和 `.usda`。
- Git LFS 只管理少量稳定的 `.usdc`、`.usdz` 和 `.onnx`。
- Shared Data 管理 checkpoint、dataset、rosbag、日志、视频以及大型或生成的 USD。

Shared Data 不是 Git 主仓库的一部分。Git 主仓库不跟踪指向 Shared Data 的符号链接。

## Linux ext4 区

适合：

- Python / C++ 源码
- ROS 2 workspace
- colcon build
- Git
- Isaac Lab 源码
- 脚本
- symlink
- Linux 权限
- 可执行文件

这些东西依赖 Linux 原生文件系统特性，因此应该放 ext4。

---

## NTFS 共享区

适合：

- 数据集
- 模型
- checkpoint
- rosbag
- USD 归档
- 视频
- 日志
- PDF / Word / Excel
- Windows 和 Linux 都要看的文件

这些通常是“大文件、结果文件、共享文件”。

---

# 4. `apps/`：AstrEX 应用层

```text
apps/
├── AstrBot/
├── AstrBotEX/
├── skill_registry/
└── adapters/
```

这一层位于：

```text
用户 / LLM
    ↓
apps
    ↓
ROS 2
    ↓
机器人
```

---

## 4.1 `apps/AstrBot/`

用途：

> 用户交互、聊天、LLM 接入。

它主要负责：

```text
用户说话
   ↓
理解语言
   ↓
调用 LLM / VLM
   ↓
形成高层任务
```

例如：

```text
用户：
“帮我把桌上的杯子拿过来”
```

AstrBot 可以负责理解这句话。

### 这里适合放

- AstrBot 本体
- AstrBot 插件
- LLM API
- 对话接口
- 用户输入处理

### 不应该放

- 机械臂 PID
- MoveIt 控制
- CAN 驱动
- 关节控制代码

---

# 5. `apps/AstrBotEX/`：AstrEX 的核心调度程序

这是整个 AstrEX 项目最重要的目录之一。

它负责：

```text
任务管理
状态管理
任务规划
动作检查
技能调度
失败处理
重新规划
```

以后可以逐步扩展成：

```text
AstrBotEX/
├── planner/
├── runtime/
├── state/
├── validation/
├── execution/
├── feedback/
└── api/
```

### 举例

用户要求：

```text
“把杯子拿给我”
```

AstrBotEX 可能产生：

```text
Navigate(table)
Pick(cup)
Navigate(user)
Place(cup)
```

然后再检查：

```text
Pick(cup)

✓ 机器人是否 READY
✓ 杯子是否存在
✓ 视觉信息是否过期
✓ 当前机械臂是否可达
✓ 是否满足安全约束
```

通过后，才交给 ROS 2 执行。

---

# 6. `apps/skill_registry/`：机器人“会什么”

这是 AstrEX 的技能字典。

它负责定义：

```text
Navigate
Pick
Place
OpenDrawer
DetectObject
Stop
```

也就是说：

> LLM 不直接操作关节，而是选择技能。

例如：

```yaml
name: Pick

parameters:
  object_id: string

preconditions:
  robot_ready: true
  gripper_empty: true
  object_visible: true

timeout: 10

results:
  - SUCCESS
  - OBJECT_NOT_FOUND
  - IK_FAILED
  - GRASP_FAILED
```

这里以后主要放：

- Skill 定义
- 参数格式
- 前置条件
- Action Contract
- 失败代码
- timeout
- 安全限制

### 最重要的理解

```text
LLM
 ↓
Skill
 ↓
ROS2
 ↓
控制器
 ↓
机器人
```

LLM 不应该直接产生 CAN 帧或关节电机命令。

---

# 7. `apps/adapters/`：不同机器人怎么执行同一个技能

AstrEX 可能以后控制：

- Isaac Sim
- Franka
- 机械臂小车
- 人形机器人
- CAT390F

但高层都可能使用：

```text
Pick(cup)
```

真正的执行方法却不同。

因此：

```text
adapters/
├── isaac_sim/
├── franka/
├── humanoid/
├── mobile_base/
└── cat390f/
```

可以理解为：

```text
统一技能
  ↓
Adapter
  ↓
不同机器人平台
```

这也是以后 AstrEX 做“跨机体”最关键的一层。

---

# 8. `ros2_ws/`：ROS 2 工作区

```text
ros2_ws/
└── src/
```

以后编译：

```bash
cd ~/Projects/AstrEX_project_main/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

编译后会自动出现：

```text
ros2_ws/
├── src/
├── build/
├── install/
└── log/
```

其中：

```text
src/
```

才是你自己真正维护的源码。

---

## 8.1 不要 Git 管理这些目录

```text
build/
install/
log/
```

因此 `.gitignore` 应该写：

```gitignore
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

---

# 9. ROS 2 包以后怎么拆？

建议逐渐发展成：

```text
ros2_ws/src/
├── astrex_interfaces/
├── astrex_bridge/
├── astrex_control/
├── astrex_perception/
└── astrex_bringup/
```

---

## 9.1 `astrex_interfaces/`

负责定义 ROS 2：

```text
msg
srv
action
```

例如：

```text
action/
├── Pick.action
├── Place.action
└── Navigate.action
```

以后所有模块都依赖这套接口。

---

## 9.2 `astrex_bridge/`

负责：

```text
AstrBotEX
   ↕
ROS 2
```

例如：

```json
{
  "skill": "Pick",
  "object": "cup_01"
}
```

转换成：

```text
ROS2 Pick Action Goal
```

---

## 9.3 `astrex_control/`

负责机器人运动控制相关内容：

- MoveIt 2
- ros2_control
- trajectory
- controller
- joint command
- safety limits

---

## 9.4 `astrex_perception/`

以后做感知时使用：

```text
Camera
 ↓
Detection
 ↓
Pose Estimation
 ↓
Object State
```

例如：

```text
/astrex/objects
```

---

## 9.5 `astrex_bringup/`

专门负责“一键启动”。

以后可能：

```bash
ros2 launch astrex_bringup sim.launch.py
```

就启动：

```text
perception
control
bridge
state manager
```

真机：

```bash
ros2 launch astrex_bringup real_robot.launch.py
```

---

# 10. `sim/`：Isaac Sim / 仿真区域

```text
sim/
├── scripts/
├── config/
├── assets/
│   ├── source/
│   └── versioned/
└── usd_work/
```

这一层只负责“虚拟机器人世界”。

---

## 10.1 `sim/scripts/`

适合：

- 启动 Isaac Sim
- 创建场景
- 加载机器人
- 配置传感器
- ROS Bridge 初始化
- 自动测试

例如：

```text
launch_isaac.py
spawn_robot.py
setup_ros_bridge.py
run_pick_demo.py
```

---

## 10.2 `sim/config/`

Isaac Sim 专用配置。

例如：

```text
physics.yaml
camera.yaml
robot.yaml
scene.yaml
domain_randomization.yaml
```

例如：

```yaml
physics:
  dt: 0.01

camera:
  width: 640
  height: 480
```

---

## 10.3 `sim/assets/`

存与源码版本相关的小型仿真资产。

例如：

```text
assets/
├── source/       # 普通 Git：.usda、URDF、Xacro、YAML
└── versioned/    # Git LFS：少量稳定 .usdc、.usdz、.onnx
```

大型或生成的 USD 放到 `/data/shared/AstrEX_project_data/simulation/`。

---

## 10.4 `sim/usd_work/`

这是实验区。

可以放：

```text
cat390f_test.usd
collision_test.usd
kitchen_v2_tmp.usd
```

可以随便改。

如果文件是小型文本 `.usda`，将它整理到：

```text
sim/assets/source/
```

如果文件是大型或生成的 USD，将它整理到 Shared Data。

---

# 11. 顶层 `config/`

这里是整个 AstrEX 的系统配置。

例如：

```text
config/
├── astrex.yaml
├── llm.yaml
├── robot.yaml
├── skills.yaml
└── logging.yaml
```

例如：

```yaml
observation:
  max_age_ms: 500

execution:
  max_retry: 2

safety:
  require_state_match: true
```

---

# 12. 顶层 `scripts/`

这里放“整个项目级别”的 shell / Python 工具。

例如：

```text
scripts/
├── start_sim.sh
├── start_ros.sh
├── start_astrx.sh
├── build_ros.sh
├── run_tests.sh
└── collect_logs.sh
```

区别：

```text
sim/scripts/
→ 只管 Isaac Sim

scripts/
→ 管整个项目
```

---

# 13. `runtime/`：当前运行产生的临时文件

```text
runtime/
├── logs/
└── rosbag_tmp/
```

核心原则：

> runtime 里的文件应该是“删掉也不会毁掉项目”的。

---

## 13.1 `runtime/logs/`

保存当前正在运行的日志。

例如：

```text
planner.log
execution.log
task.log
ros.log
```

推荐：

```text
runtime/logs/2026-09-16_exp001/
```

实验结束后再归档到共享盘。

---

## 13.2 `runtime/rosbag_tmp/`

当前正在录制的 rosbag。

例如：

```text
/camera
/joint_states
/tf
/astrex/skill_status
```

实验结束后迁移到：

```text
/data/shared/AstrEX_project_data/logs/rosbag/
```

---

# 14. `docs/`

这里放“跟代码一起维护的开发文档”。

例如：

```text
architecture.md
developer_setup.md
skill_contract.md
ros2_interfaces.md
sim_setup.md
```

这些应该进入 Git。

---

# 15. `tests/`

以后非常重要。

这里可以测试：

```text
旧观测是否被拒绝
错误 Action 是否被拦截
Skill 参数是否合法
状态变化以后是否重新规划
ROS2 Action 是否能正确执行
```

例如：

```text
tests/
├── test_action_contract.py
├── test_skill_registry.py
└── test_state_freshness.py
```

---

# 16. `tools/`

放开发辅助程序。

例如：

```text
log_analyzer.py
bag_converter.py
dataset_converter.py
usd_checker.py
```

这些不是 AstrEX 主程序，而是辅助开发。

---

# 17. 共享盘 `/data/shared/AstrEX_project_data/`

这一层的原则是：

> 长期保存 + 大文件 + Windows/Linux 共享。

---

# 18. `datasets/`

建议：

```text
datasets/
├── raw/
├── processed/
├── annotations/
└── external/
```

### raw

原始采集数据。

### processed

清洗 / 转换后的数据。

### annotations

标注文件。

### external

外部下载的数据集。

---

# 19. `models/`

建议：

```text
models/
├── checkpoints/
├── pretrained/
├── exported/
└── legacy/
```

### checkpoints

训练过程中的模型。

### pretrained

下载的预训练模型。

### exported

真正准备部署的：

```text
ONNX
TensorRT
TorchScript
```

### legacy

以前项目遗留模型。

例如你旧的 CAT390F PPO 模型可以放这里。

---

# 20. `logs/`

```text
logs/
├── app/
├── isaac/
└── rosbag/
```

---

## `logs/app/`

长期保存 AstrEX：

- planner
- task
- action proposal
- validation
- replan

以后做论文实验非常重要。

---

## `logs/isaac/`

保存：

- Isaac Sim console log
- PhysX warning
- simulation output

---

## `logs/rosbag/`

保存正式 rosbag。

建议按：

```text
logs/rosbag/
└── 2026-10/
    ├── exp_001/
    ├── exp_002/
    └── exp_003/
```

---

# 21. `exports/`

这里放：

> 已经整理好，准备交给别人或 Windows 使用的结果。

例如：

- ONNX
- TensorRT
- CSV
- Excel
- 实验图
- 比赛材料
- Demo 视频
- 部署包
- zip

---

# 22. `documents/`

这里放项目资料：

- PDF
- Word
- Excel
- 申请书
- 论文
- 会议材料
- 技术资料

和 `docs/` 区别：

```text
main/docs/
→ 开发文档，跟代码走

shared/documents/
→ 项目资料、论文、Office 文件
```

---

# 23. 我有一个新文件，到底应该放哪里？

可以用这个表。

| 文件 | 推荐目录 |
|---|---|
| Python 源码 | `main/` |
| C++ 源码 | `main/` |
| ROS package | `ros2_ws/src/` |
| `.action/.msg/.srv` | `astrex_interfaces/` |
| Isaac Sim Python 脚本 | `sim/scripts/` |
| 小型文本 `.usda` | `sim/assets/source/` |
| 少量锁定版本的 `.usdc/.usdz/.onnx` | `sim/assets/versioned/` |
| 大型或生成的 USD | `shared/simulation/` |
| 临时 USD | `sim/usd_work/` |
| 系统配置 | `config/` |
| 当前日志 | `runtime/logs/` |
| 当前 rosbag | `runtime/rosbag_tmp/` |
| 正式 rosbag | `shared/logs/rosbag/` |
| 数据集 | `shared/datasets/` |
| checkpoint | `shared/models/checkpoints/` |
| 预训练模型 | `shared/models/pretrained/` |
| ONNX/TensorRT | `shared/models/exported/` |
| 旧模型 | `shared/models/legacy/` |
| PDF/Word | `shared/documents/` |
| 最终结果 | `shared/exports/` |

---

# 24. 初学者最重要的扩展原则

不要一次建很多空文件夹。

只有当项目真的出现一种新功能时再增加目录。

例如：

现在只有 ROS 通信：

```text
ros2_ws/src/astrex_interfaces/
```

以后开始视觉：

```text
+ astrex_perception/
```

以后开始真机启动：

```text
+ astrex_bringup/
```

以后增加人形机器人：

```text
adapters/
+ humanoid/
```

这样不会一开始就把自己绕晕。

---

# 25. AstrEX 的整个数据流

以后可以一直用这张图理解项目：

```text
User
 │
 ▼
AstrBot
 │
 ▼
AstrBotEX
 │
 ├── State
 ├── Planner
 ├── Validator
 └── Skill Registry
 │
 ▼
Adapter
 │
 ▼
ROS 2
 │
 ├── Perception
 ├── MoveIt
 ├── ros2_control
 └── Actions
 │
 ▼
Isaac Sim / Real Robot
 │
 ▼
Feedback
 │
 └─────────────→ AstrBotEX
```

---

# 26. 你现在应该做什么？

不要急着同时开发整个系统。

推荐最小路线：

```text
Step 1
ROS2 基础通信
/clock
/joint_states

↓

Step 2
ROS2 → Isaac Sim
发送 joint command

↓

Step 3
建立 astrex_interfaces

↓

Step 4
定义第一个 Skill
Pick.action

↓

Step 5
写一个假的 Pick Action Server

↓

Step 6
AstrBotEX 调 Pick

↓

Step 7
再把假的 Pick 换成 MoveIt / Isaac Sim 真抓取

↓

Step 8
加入 Action Contract

↓

Step 9
加入失败反馈和 Replan
```

不要一开始就同时上：

```text
LLM + VLM + MoveIt + 人形机器人 + RL + Sim2Real
```

否则很难判断错误在哪一层。

---

# 27. 只需要记住的 5 条规则

如果上面的内容暂时记不住，只记下面五条：

1. **代码放 `AstrEX_project_main`。**
2. **大数据和模型放 `AstrEX_project_data`。**
3. **ROS 代码放 `ros2_ws/src`。**
4. **Isaac Sim 相关放 `sim/`。**
5. **当前运行产生的临时文件放 `runtime/`。**

遇到不知道放哪里的时候，先按照这五条判断。

---

# 28. 一句话总结

```text
AstrEX_project_main
= 怎么运行 AstrEX

AstrEX_project_data
= AstrEX 运行产生和使用的数据
```

只要始终保持这个边界，项目规模扩大以后也不会很乱。

---

# 29. Isaac 基线入口与文件归位（2026-09-17）

当前 AstrEX Isaac 开发基线是 `isaaclab232_test`（Isaac Sim 5.1、Isaac Lab v2.3.2、ROS 2 Jazzy domain 63），版本期望在 `config/isaac_baseline.env`，验收与限制见 [开发基线](ISAAC_51_DEV_BASELINE.md)。

顶层 `scripts/` 只放正式用户入口：

```text
scripts/check_isaac_env.sh
scripts/start_isaac_rl.sh
scripts/start_isaac_ros.sh
```

实现分层：

| 内容 | 位置 |
|---|---|
| 入口包装脚本 | `scripts/*.sh` |
| 启动器私有实现（不要直接调用） | `scripts/lib/isaac_common.sh`、`scripts/lib/isaac_entry.py` |
| Isaac 场景 / OmniGraph / 物理 / ROS 运行逻辑 | `sim/scripts/run_ros_cartpole.py`、`sim/scripts/ros_domain_check.py`、`sim/scripts/rl_official.py` |
| 回归测试（系统 ROS / Isaac 两侧分开） | `tests/isaac/` |

回归与诊断脚本不要放进 `scripts/`；运行时产物不要进 Git（`/data/shared/AstrEX_project_data/logs/isaac/` 保存运行证据）。

注意：`start_isaac_ros.sh` 启动前的严格 domain preflight 是 **cutover 验收门禁**，不是日常开发的永久阻塞条件；日常策略目标是以 endpoint conflict 为核心（`/clock`、`/joint_states`、`/joint_command`、controller/action/service 实际冲突才阻断）。细节见 [开发基线](ISAAC_51_DEV_BASELINE.md)。
