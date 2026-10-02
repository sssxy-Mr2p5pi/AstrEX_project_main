# AstrEX CartPole MVP V4：人话版开发说明

> 目标：读取固定 command.json，完成 MoveCart 后读取新状态，再运行 BalanceHold。两步都成功，整个 command 才成功。
>
> 按本文逐步开发，前一步通过后再进入下一步。

---

# 1. 先看目标和目前进度

标准 ROS Topic 与 Isaac/PhysX 控制链已有验收记录，见[Isaac 开发基线](ISAAC_51_DEV_BASELINE.md)。

StateCache、控制器、Skill Action Server、Grounder、Runner、SafetyGuard 仍需开发验证。接口目录尚无 `MoveCart.action`、`BalanceHold.action`。下文是开发目标，示例数字不是实测成绩。

固定任务顺序：

1. MoveCart：边稳定摆杆，边将滑车移到目标。
2. BalanceHold：从新状态开始，在目标位置稳定指定时间。

正式数据链如下，当前 Skill 只选择一个 controller：

```text
command.json
↓
Command Runner
↓
当前 Skill Grounder
↓
typed ROS 2 Action Goal
↓
CartPole Skill Server
├── MoveCart Action Server → MoveCart controller
└── BalanceHold Action Server → BalanceHold controller
↓
/astrex/raw_joint_command
↓
SafetyGuard
↓
/joint_command
↓
Isaac ROS 2 Bridge
↓
IsaacArticulationController
↓
PhysX
↓
/joint_states
↓
StateCache
    └──→ Skill Action Server / Runner / Grounder
```

PhysX 计算运动，GUI 显示场景。关节反馈送回系统 ROS 侧，供控制器计算下一次施力。

---

# 2. 这次 MVP 到底在证明什么

先完成第一步，再用新状态启动第二步，并判断各步成败。

## 2.1 先认清四个状态量

| 名称 | 含义 | 单位 |
|---|---|---|
| `x` | 滑车在轨道上的位置 | m |
| `x_dot` | 滑车沿轨道移动的速度 | m/s |
| `theta` | 摆杆相对直立位置的角度 | rad |
| `theta_dot` | 摆杆角速度 | rad/s |

`slider_to_cart` 对应滑车，`cart_to_pole` 对应摆杆。Step 2 实测杆角零点和正负方向，角度使用弧度。

## 2.2 第二步必须用第一步之后的新状态

例如开始时：

```text
x = 0.00 m
theta = 0.00 rad
```

MoveCart 完成后可能得到：

```text
x = 0.497 m
x_dot = 0.018 m/s
theta = 0.035 rad
theta_dot = -0.06 rad/s
```

BalanceHold 使用移动后的真实状态，不能假设仍为零。是否合格由容差决定，新状态规则见第 7 节。

## 2.3 Success 要看连续稳定

滑车瞬间经过 0.50 m，不能算 MoveCart 成功。四个状态量必须一起连续合格。BalanceHold 的 3 秒也指连续保持时间，不能累加零散的合格时段。

---

# 3. 每个模块到底干什么

## 3.1 command.json：安排任务

它将请求写成有序列表供 Runner 读取，`move_cart`、`balance_hold` 对应两个 Skill。

沿用下面的请求示例：

```json
{
  "skills": [
    {
      "skill": "move_cart",
      "goal": {
        "target_position_m": 0.5
      }
    },
    {
      "skill": "balance_hold",
      "goal": {
        "duration_sec": 3.0
      }
    }
  ]
}
```

控制频率和 LQR 参数放在本地配置中。0.5 m、3.0 s 是演示请求。

检查：名称、顺序、字段、单位正确；非法请求在发 Goal 前报错。

## 3.2 Runner：按顺序执行任务

Runner 接收任务列表、Action Result 和状态，调用当前 Grounder、发 Goal、等待结果。输出当前步骤和 command 结果，不计算或发布控制力。

第一版不自动重试。MoveCart 失败就结束 command；成功后先等合格的 S1，再调用第二个 Grounder。

检查：MoveCart 成功结果、S1、BalanceHold Goal 必须按此顺序出现。

## 3.3 StateCache：保存最新有效状态

它订阅 `/joint_states`，输出四状态快照，供上层模块读取。按 `JointState.name` 查索引，再读对应的 `position`、`velocity`，不能固定使用数组第 0 项。

第一版检查：

- 两个关节名称存在，且能够确定唯一索引。
- position、velocity 数组都有对应元素。
- 四个状态量均为有限数，即没有 NaN、Inf。
- 状态没有超过配置允许的失效时间。

保存 `header.stamp` 判断状态先后，保存本地单调接收时间判断新鲜度。前者来自仿真，两类时间不能直接混减。

快照的四个量必须来自同一消息。无效消息不能刷新有效接收时间。状态过期时，不能生成可执行 Goal，也不能继续控制。

检查：打印状态和时间戳，测试关节换序、缺失、无效数和停止更新。

## 3.4 Grounder：准备本次执行的 Goal

Grounder 每步开始前调用一次，将请求、最新状态和配置整理为 Goal，不控制或发布 joint command。

MoveCart Grounder：

```text
输入：target = 0.5 m、有效状态 S0、本地配置
处理：检查目标和初态，选择控制边界与成功条件
输出：目标位置、成功容差、稳定时间、允许力/速度、执行超时
```

BalanceHold Grounder：

```text
输入：有效状态 S1、上一步目标 0.5 m、保持时间 3 s、本地配置
处理：检查 S1 是否适合开始保持，整理本次保持参数
输出：hold_position = 0.5 m、保持时间、成功容差、控制边界、执行超时
```

S1 用于检查初态、确定参数；目标仍可为 0.5 m，无须改成 `S1.x`。

检查：同一请求搭配不同状态，确认实际使用了新状态。无效或不适合的初态不能生成可执行 Goal。

## 3.5 CartPole Skill Server：接收任务并持续控制

CartPole Skill Server 是系统 ROS 节点，提供两个 Action Server。接收字段和类型明确的 Goal，读取状态，运行当前 controller，输出力、Feedback、Result：

```text
读取最新有效状态
→ 检查超时或危险越界
→ 计算当前 Skill 的 cart force
→ 发布 /astrex/raw_joint_command
→ 发布 Action Feedback
→ 更新连续稳定计时
→ 必要时结束 Action 并返回 Result
```

两个 Action Server 共用执行占用标记，检查与设置占用必须不可分割。已有 Goal 执行时拒绝新 Goal，不排队、不抢占。

检查：分别调用 Action；执行中提交第二个 Goal 须被拒绝，结束后旧循环停止输出。

## 3.6 ROS Action 与 Isaac Bridge：分别负责什么

ROS Action 是任务通信接口：

| 部分 | 给技术员的解释 | 本 MVP 需要的信息 |
|---|---|---|
| Goal | 这次要完成什么 | 目标、控制边界、成功条件、超时 |
| Feedback | 现在做到哪里了 | 当前状态、位置误差、已连续稳定多久 |
| Result | 最后有没有完成 | 成功或失败、结束原因、完成状态的时间戳 |

完整字段在实现时定义，成功判定时间戳须传给 Runner。

两个自定义 Action 由系统侧 Skill Action Server 执行。Isaac ROS Bridge 处理标准 Topic：

```text
/joint_command → ROS2SubscribeJointState → IsaacArticulationController → PhysX
PhysX → ROS2PublishJointState → /joint_states
```

Bridge 不直接执行自定义 Action。检查：标准 Topic 链工作后，Action Server 能根据 Goal 持续输出控制力。

---

# 4. 为什么先开发 BalanceHold

给滑车施力会同时影响滑车和摆杆。如果先写只管位置的 MoveCart：

```text
离目标远 → 大力推
```

滑车可能到位，摆杆却倒了。因此先学会“站稳”，再学会“边移动边站稳”。

**开发顺序：BalanceHold Controller → MoveCart Controller。任务顺序：MoveCart → BalanceHold。**

可复用计算函数，但不能同时运行两个 Skill。

---

# 5. Controller 第一版怎么做

V4 建议先用 LQR：综合四个状态量，计算下一次滑车施力。

```text
x_error = x - x_ref
输入：[x_error, x_dot, theta, theta_dot]
输出：F_cart，单位 N
pole torque = 0
```

本轮只操纵滑车，不主动给摆杆独立 torque。LQR 系数须根据本场景模型调试，未经验证的数字不能保证稳定。

BalanceHold controller 接收固定 `hold_position` 和状态，输出保持力。检查四个状态量是否连续合格。

MoveCart controller 接收 target、当前 `x_ref` 和状态。让参考位置平滑移向目标，检查到位后速度和杆角是否一起稳定。

**MoveCart 内部同时反馈四个状态量，兼顾移动和稳定。此时 BalanceHold 尚未启动。**

控制频率、参考速度、限力、容差、稳定时间和超时均待调试。每次试验记录配置，再判断效果。

---

# 6. 开发顺序：按八步推进

步骤 3～4 限制初态、力和时长，结束清零。Step 8 通过后才算完整 MVP 验收完成。

## Step 1：复查现有 Isaac 控制链

- 要做：用现有入口复查标准 Topic 链，按已有受控试验方式测试零、正、负力。
- 产物：命令、关节反馈和运动方向的短记录。
- 验收：真实运动与反馈一致，关节和单位正确，零命令清除非零施力。
- 排错：查 Topic、消息类型、joint 名称和控制图。

结束后停止临时发布器，避免干扰后续控制器。

## Step 2：实现 StateCache

- 要做：按关节名称读取四个量，保存时间和有效标记。
- 产物：一致的状态快照。
- 验收：实测方向、杆角零点；检查数组换序、缺关节、无效数和过期状态。
- 排错：查索引、数组长度、单位、时间来源，再调控制器。

## Step 3：单独实现 BalanceHold Controller

- 要做：从接近直立的小范围初态开始，围绕固定位置计算滑车力。
- 产物：控制函数和有限时长的独立测试入口。
- 验收：四个状态量连续合格，力有限，结束清零。
- 排错：先查角度和施力方向，再查限力、周期和 LQR 系数。

倒杆触发场景 reset 时，本次试验失败；reset 后归零不能算控制成功。

## Step 4：实现 MoveCart Controller

- 要做：增加平滑参考，让滑车在保持杆稳定的同时移动。
- 产物：参考位置生成逻辑和 MoveCart 控制函数。
- 验收：最终位置、速度和摆杆状态连续满足成功条件。
- 排错：查参考是否过快、力是否长期到上限、到位后速度是否降下来。

## Step 5：包装两个 ROS 2 Action Server

- 要做：实现两个 .action，将各自控制函数接入 Skill Server，并共用执行占用标记。
- 产物：两个可调用的 Action Server。
- 验收：手动发 Goal，核对 Feedback、Result、结束清零和第二个 Goal 被拒绝。
- 排错：查参数传递、成功判定、旧控制循环是否停止。

这是待开发步骤，不能据此认为接口已经存在。

## Step 6：实现两个简单 Grounder

- 要做：将请求、当前有效状态和配置整理为 typed Goal。
- 产物：每步开始前调用一次的两个 Grounder。
- 验收：记录输入状态时间戳和 Goal；无效初态不启动，第二步使用 S1。
- 排错：查是否误用了 S0、遗漏目标、容差或控制边界。

## Step 7：实现 Runner

- 要做：按第 7 节串起两个 Skill，第一步结束后等待新状态。
- 产物：读取 JSON、顺序发 Goal、输出 command 结果的 Runner。
- 验收：顺序正确；第一步失败或 S1 等待超时，第二步未启动。
- 排错：查是否把 Goal 接收当成功、复用旧状态或提前发第二个 Goal。

## Step 8：加入最小 SafetyGuard 和 failure 测试

- 要做：接通 raw → Guard → final command，执行第 9 节检查。
- 产物：Guard 和四类异常测试记录。
- 验收：无效力、超限力、状态中断、命令中断处理正确，再测完整两步流程。
- 排错：查是否绕过 Guard、是否留下旧非零力、故障是否被错误报为成功。

---

# 7. 两个 Skill 严格顺序执行

sequential 指前一个 Skill 完整结束后才启动下一个。任何时刻最多一个 Skill Action 正在执行。

```text
读取有效初始状态 S0
↓
Ground MoveCart(S0)
↓
生成 MoveCart Goal C0
↓
MoveCart Action Server 执行
↓
成功条件连续满足，记录该状态的时间戳 t_success
↓
停止 MoveCart 控制循环，发送一次零命令
↓
返回 MoveCart SUCCESS，释放执行占用
↓
Runner 收到成功结果
↓
等待新有效状态 S1，要求 S1.stamp > t_success
↓
Ground BalanceHold(S1)
↓
生成新的 BalanceHold Goal C1
↓
BalanceHold Action Server 执行
↓
成功条件连续满足指定保持时间
↓
停止控制循环，发送一次零命令
↓
返回 BalanceHold SUCCESS
↓
Command SUCCESS
```

清理也属于当前 Skill：先停止旧循环，再向 raw Topic 发一次零命令，最后返回 Result。成功后不再控制，也不后台运行另一个 Skill 填补间隔。

`t_success` 来自成功判定的状态消息。Runner 收到结果后，等待新到达、时间戳严格更大的有效消息作为 S1，拒绝重复、乱序旧消息和过期状态。

两步之间有短暂的无 Skill 控制间隔。零力不保证摆杆站稳，必须实测间隔，并检查 S1 是否适合开始保持。

等待 S1 超时为 FAILED；无效状态或危险越界为 FAULT。S1 不满足允许初态范围时，不启动第二步，也不恢复 MoveCart 或自动重试。

---

# 8. Success 第一版怎么定义

Success Predicate 即成功条件。先确定逻辑，再实验调试阈值。

## MoveCart SUCCESS

```text
|x - target| ≤ 位置容差
AND |x_dot| ≤ 速度容差
AND |theta| ≤ 角度容差
AND |theta_dot| ≤ 角速度容差
AND 连续满足时间 ≥ 稳定时间
```

位置误差使用最终 target，不能因为暂时跟上了途中的 `x_ref` 就提前成功。

## BalanceHold SUCCESS

```text
|x - hold_position| ≤ 位置容差
AND |x_dot| ≤ 速度容差
AND |theta| ≤ 角度容差
AND |theta_dot| ≤ 角速度容差
AND 连续满足时间 ≥ duration_sec
```

示例 `duration_sec = 3 s` 表示连续保持 3 秒。实际容差仍要调试。

所有条件满足时开始或继续计时。任一成功条件不满足，连续计时归零；只要没有危险越界且未超时，控制仍可继续。

稳定时间按有效状态的仿真时间戳累计，重复缓存和断流不能算稳定。执行和等待超时使用本地单调计时，保证状态停止更新时也能触发。

执行超时为 FAILED；无效状态、危险越界或安全故障为 FAULT。成功条件与故障同时出现时，优先处理故障。

---

# 9. SafetyGuard 第一版只保留最小功能

SafetyGuard 接收 raw command 和状态，检查后发布最终 `/joint_command`。Skill Server 只发 `/astrex/raw_joint_command`。两侧使用标准关节消息，滑车力对应 `slider_to_cart` 的 effort，单位 N。

| 检查 | 第一版处理 |
|---|---|
| finite check | 拒绝 NaN、Inf，输出零力 |
| force limit | 将有限力限制在允许的正负最大值内 |
| state freshness | 状态过期时停止放行，输出零力 |
| command watchdog | 超过允许时间未收到有效新命令时输出零力 |
| fault | 输出零力，当前任务不能报告成功 |

watchdog 需要自己定时检查，即使没有新命令回调也能输出零力。收到无效命令不能刷新有效命令时间。

检查：对比 raw 和 final Topic，超限力被限制，无效力或数据超时使最终力归零。

故障须结束当前 Skill，并让 Runner 得知失败，不能清零后仍报 SUCCESS。具体字段在实现时确定。

发送零力只表示停止施力，不保证倒立摆继续站稳。限力值和超时值都要在本机试验中配置并记录。

---

# 10. 第一版只保留五个业务状态

| 状态 | 含义 |
|---|---|
| IDLE | 等待任务，没有正在执行的 Skill |
| RUNNING | 当前任务正在执行 |
| SUCCESS | 成功条件已满足，任务已结束 |
| FAILED | 未完成任务，例如执行或等待超时 |
| FAULT | 状态或控制安全检查失败，要求停止输出力 |

ACTIVE 仅表示“正在执行”，对应业务 RUNNING。这五个业务状态不替换 ROS Action 原有协议状态。

等待 S1 时，command 仍可为 RUNNING，但两个 Skill 都不执行。FAILED 或 FAULT 均结束本次任务，不继续第二步。

---

# 11. 第一轮 MVP 成功标准

下面是**预期演示**，数字用于解释流程，不代表已实测通过：

```text
读取 command.json
MoveCart：target = 0.5 m
x：0.00 → 0.18 → 0.34 → 0.46 → 0.49 → 0.50
四个状态量连续合格
→ 停止 MoveCart 控制、清零
→ MoveCart SUCCESS

收到一条更新的有效状态 S1
→ Ground BalanceHold(S1)
→ 发出新的 Goal
BalanceHold：hold_position = 0.5 m
四个状态量连续合格 3 s
→ 停止控制、清零
→ BalanceHold SUCCESS
→ Command SUCCESS
```

GUI 辅助观察滑车移动、到位、保持和摆杆状态。验收还须保留：

- 两个 Action 开始和结束的顺序，确认没有同时执行。
- MoveCart 的成功状态时间戳、S1 时间戳及 S1 的有效性。
- 两个 Grounder 使用的状态、生成的 Goal 和最终 Result。
- 成功时四个状态量及连续稳定时间。
- 第一步失败、等待 S1 超时或第二步初态不合格时，第二步未启动。
- 非有限力、超限力、状态中断、命令中断四类测试的结果。

用现有终端或简单测试记录保存证据即可。成功流程和失败测试都有证据后，才能填写本轮验收结果。

---

# 12. 如果失败，先看哪一层

按这个顺序排查，每次先解决当前发现的问题：

1. StateCache：索引、单位、符号、时间是否正确？
2. BalanceHold：能否持续稳定，是否因 reset 才归零？
3. MoveCart：参考是否过快，到位后速度是否合格？
4. Controller：力是否有限、长期限幅，结束后是否停发？
5. SafetyGuard：为何拦截，最终命令是否有额外发布者？
6. Action Server：参数是否正确，Result 是否过早，Goal 是否互斥？
7. Grounder 与 Runner：是否用了 S1，是否严格顺序？

先检查反馈，再检查控制，最后检查任务编排。不要为了让示例“看起来成功”而直接放宽所有容差。

---

# 13. 以后怎么扩展

主链通过后，可替换任务输入、机器人和控制算法。每次替换仍按“当前状态 → Goal → 执行 → 结果”重新验收。

---

# 14. 一句话记住当前开发顺序

```text
验证 Topic 链
→ StateCache
→ BalanceHold Controller
→ MoveCart Controller
→ 两个 Action Server
→ 两个 Grounder
→ 顺序 Runner
→ 最小 SafetyGuard 与 failure 测试
```

验收主线：MoveCart 结束 → 新状态 → BalanceHold → Command SUCCESS。
