# AstrBotEX 与 A.E.B. 功能现状

核对与同步日期：2026-10-01。AstrBotEX 已同步固定提交 `c9624a4`。本轮只同步、运行已有测试和整理后续计划，没有开发 B08/B09、机械臂、Laya 或训练功能，也没有 commit/push。

## 1. 版本与验证状态

| 组件 | 本地版本 | 本轮结果 |
|---|---|---|
| AstrBotEX | [`c9624a439874740cde4acc347c979419af120d05`](https://github.com/Steven-Wang-120/AstrbotEX/commit/c9624a439874740cde4acc347c979419af120d05) | 116 个新增文件、19 个修改文件已同步；保留三份本地 Markdown 格式差异 |
| A.E.B. | [`7f9790d823dc42af91cbc928cdb98c48f3eef737`](https://github.com/Steven-Wang-120/A.E.B./commit/7f9790d823dc42af91cbc928cdb98c48f3eef737) | 未更新 submodule，未安装/启用到 AstrBot |
| AstrBot | `ab42c0d9b726d82ad0f9563e04c53a4460c00d61` | 官方 submodule 保持不变 |

状态用语：**源码已有**表示能在固定版本找到实现；**本机已验证**必须附测试范围；**尚未接线**表示相关接口存在但完整链未连通；**待开发**表示当前没有实现。

本机完整测试首轮为 **476 项：470 通过、1 失败、5 环境错误、0 跳过**。5 项 ROS 环境错误处理后，定向重跑 **5/5 通过**。剩余停止状态断言失败未修复、未刷过，因此不能声称全量回归通过。详见[本轮报告与原始证据](ASTRBOTEX_SYNC_20261001_RESULT.md)。

## 2. 上层非控制能力

| 模块 | 源码能力 | 本轮证据与尚缺部分 |
|---|---|---|
| A.E.B. / AstrBot | AstrBot 平台适配、EX 上下文、动作提案工具、视觉缓存工具、STT/TTS 代理 | 版本未变；本轮未重跑 A.E.B.，未做 AstrBot 端联调 |
| 文本通道 | A.E.B. ROUTER 与 EX DEALER；默认 ZMQ 8766，握手与结构化消息 | EX 连接测试覆盖传输；8766 不是 HTTP 端口 |
| 音频通道 | 默认 ZMQ 8767，STT 输入/TTS 输出及提供者状态 | 相关框架测试通过；真实语音与模型服务未验证 |
| 视觉通道 | 默认 ZMQ 8768；`vision.json.publish` 与 `vision.jpeg.publish`，旧通道兼容 | 视觉缓存按工具读取，不自动加入每次 LLM 请求；相机到 AstrBot 全链未验证 |
| 内置 YOLO | 主进程轻量包装、独立推理 worker、JSON/JPEG 转发 | 保持禁用；未下载权重、未运行推理或摄像头 |
| 插件与能力目录 | manifest 校验、加载/配置、动作与观测声明、观测说明 | registry/catalog/lifecycle 现有测试通过；B03 工程模拟插件在独立 EXplugin 仓库，未导入 |
| 环境与页面 | normal/ros2 模式、配置版本、异步切换、图发现、绑定诊断与环境页 | 环境/API 测试及 5 项原生 ROS 测试通过；本轮未做浏览器 UI 验收 |
| 连接与运行管理 | HTTP/SSE、WebSocket/ZeroMQ、runtime 生命周期、诊断状态 | 原有框架测试覆盖；没有启动用户实际运行实例 |
| 实例快照 | profiles/plugins 配置备份与恢复、执行状态另存 | 已有 backup/storage 测试通过；管理密钥与权限能力仍属 B08 |
| 部署 | `Dockerfile.ros2`、ROS 配置/接口包、Python 元数据改为 >=3.10 | 已构建既有 ROS 示例消息包；未构建 Docker，未验证 Python 3.10/ARM64 |
| 上下游协调 | 冻结 Goal/反馈合同、可信服务方法和 transport 解析 | A.E.B. 任务队列、公开回复隔离及外部 Goal/反馈接线未包含在本次固定提交中 |

先前 2026-09-23 的 100 项 EX 测试和 A.E.B. 传输结果属于旧版本历史，不能代替本轮验证。A.E.B. 旧完整测试曾缺少 `mcp` 依赖，本轮未修复或重试该环境。

## 3. 控制端的主要变化

### 3.1 定向动作通道

新增 `core/actions/`、冻结合同和动作目录。decision 模式使用 **DecisionService → Dispatcher → 对应 owner 的 Actor → 插件回调**。TopicBus 仍承担观测/诊断，不能承担可靠动作命令与终态。

legacy 与 decision 控制路径隔离，不代表旧路径已从仓库删除。`build_server()` 默认仍为 `control_mode=legacy`、decision disabled、调度门关闭；源码同步不会自动启用动作。

| 状态 | 表示什么 | 不代表什么 |
|---|---|---|
| admitted | 框架受理并进入动作处理 | 插件已开始运动 |
| accepted | 目标插件受理命令 | ROS 控制器已受理 |
| running | 插件报告正在执行 | 成功判据已满足 |
| succeeded | 关联动作有可信成功报告 | 模型选择天然正确 |
| canceled | 动作取消且提供结构化停止证据 | 仅收到 cancel 就算停止 |
| unknown / timed_out / failed | 不确定、到期或失败 | 资源一定可以立即释放 |

Actor start/cancel 回调默认预算 20 ms。规划、推理、等待 ROS 结果必须转到异步 worker。

### 3.2 持久账本与恢复

Ledger 位于 `data_root/execution/actions.sqlite3`，保存命令、事件、资源和停止证据。幂等与资源锁由框架维护。旧 `profiles/default/actions.sqlite3` 首次迁移使用 SQLite backup API，包含已提交 WAL；不能只复制主数据库文件。

execution 不属于 profiles/plugins 配置快照。恢复配置不能回滚动作事实、重新启用旧 Goal 或重放旧命令。重启会产生新会话，未确认动作进入 unknown/复核流程。

取消是异步过程。StopEvidence 需要 command_id、stopped、source 和 reference。失败或 unknown 动作的资源可能继续保留，必须通过受控停止复核释放。

### 3.3 Goal、观测和候选失效

GoalManager 保持一个活动 Goal 和至多一个待替代 Goal。替换先撤销旧执行门，完成旧动作停止与证明后再激活新 Goal。模型不选择下一步 Goal；任务完成依赖当前版本的 required_success_actions。

ObservationStore 保存原始来源 JSON，并附接收单调时钟、来源 epoch、seq、说明 hash 和目标引用。新鲜帧不能为旧请求续期；配置/目标/来源/环境/插件变化会使旧结果失效。

DecisionService 使用独立控制和后端线程，最多一项 live 请求，观测变化合并处理。停止和租期检查不等待后端模型完成。

当前 Dispatcher context 存在身份约束：有未结束动作时不能追加独立 start 轮次，即使资源不同。因此首版底盘与机械臂顺序执行，不开展全身并行动作。

### 3.4 Jev 能力限制

可信后端工厂只有 Mock/Jev。Jev 的 `execution_allowed=False` 是框架限制，与密钥、供应商返回或配置按钮无关。默认 disabled、live_http 关闭，固定模型 `jev-1.13.0`，只支持 disabled/shadow。

Jev 默认最小请求间隔 500 ms，不能把它当作 100 Hz 控制环。本机测试使用 fake transport 或 loopback HTTP，没有访问真实供应商、没有付费调用。

上游离线评测的 100 条数据是合成场景；rule/stub 是脚本，不是 Jev/Laya/LLM。其结果只能验证评测管线，不能作为机器人模型成功率。

### 3.5 原生 ROS 端口已经接通

旧版本 facade 仅接内部 TopicBus 的描述已过时。当前插件端口已连接原生 rclpy publisher/subscription，具备队列、QoS、绑定代次、过期回调隔离和环境切换清理。

本机 5 项 DDS 测试覆盖跨进程双向传输、多个 owner、自定义嵌套消息、缺类型隔离、QoS 修复、高频队列和 20 次环境切换。仅使用隔离 Domain 173 的测试 String/Target 消息，没有发送关节或速度命令。

`queued/tx_published` 只证明入队/本地发布，不证明远端受理，更不证明物理停止。完整 ROS Action 客户端 facade、机器人业务 ACK、轨迹执行和本体 watchdog 仍需后续控制端适配。

## 4. B08/B09 与 Laya 的分工

| 项目 | 当前状态 | 后续交付 |
|---|---|---|
| B08 | 待开发，可复用 B02/B04 服务 | 管理配置、密钥、访问校验、probe、实际请求追踪、异步停止状态 |
| B09 | 待开发，可复用现有 dashboard | 一级决策页，区分 EX 决策与控制端轨迹评分 |
| 原始 Laya | 未安装、未接入、未实测 | 独立 worker + Adapter，选择 Grounder 候选 |
| ROS 机械臂闭环 | 通信基础已测，执行能力待开发 | Guard、网关、标准 Action、Isaac 运动、反馈、StopEvidence |
| Laya 微调 | 尚未采集/训练 | 原始模型闭环通过后，100 组样本、一次小规模微调与独立比较 |

`snapshot()` 当前返回最近构建快照；Jev last_record 没有完整实际请求。B08 必须在真实调用边界补追踪，不能靠页面重建文本冒充“实际输入”。当前响应的通配 CORS 和旧控制入口也必须纳入 B08 访问检查。

冻结 Goal 每个 action_id 只有一份固定参数。多条轨迹放在控制插件内部：

```text
固定脚本正式 Goal → EX 确定性调度 → 控制插件
    → Grounder/PyRoKi 候选 → Laya 选择 → Guard → ROS 2 → Isaac
    → 反馈/物理结果 → Ledger → B09
```

上游云端 Jev 保持 shadow，Laya 不冒充 EX 已有 backend。以后由 A.E.B. 替换任务脚本时，执行链不变。

## 5. 已确定与待处理的事项

已确定：保持目录；本机/SSH 管理；直接机械臂；冻结上游契约；控制端多轨迹评分；原始模型先过 ROS 实际闭环，再小规模微调；本轮不开发、不 commit/push。

**同步与计划交付没有待决阻塞。** 本机尚有一项停止状态测试失败。后续执行开发前，需要明确由上游修复还是另行安排本地诊断/修复；本轮只记录证据，没有放宽测试或修复源码。该失败不能直接推导为机器人未停止，也不能忽略为已通过。

机器人资产、传感器升级、付费算力仅在后续出现真实阻塞时再讨论。相关任务见[施工索引](AstrEX_workingTree/00_施工总索引与派工规则.md)、[移动机械臂详细计划](MOBILE_MANIPULATION_PYROKI_MVP_PLAN.md)。
