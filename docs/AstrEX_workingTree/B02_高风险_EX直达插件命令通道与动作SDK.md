# B02｜高风险｜EX直达插件命令通道与动作SDK

作用：实现决策者到插件的明确控制通道与执行事实账本，绕开TopicBus命令广播。预计5–8人日。依赖B00。必须强agent实施与复核。

## 1. 当前事实与修改位置

| 现有位置 | 问题/基础 | 本批修改 |
|---|---|---|
| `core/local_plugins.py::ActionDeclaration/_parse_actions/_validate_actions` | action要求topic，描述能力有限 | 支持B00 v2动作声明；topic只对旧版要求 |
| 同文件 `PluginContext` | 只有bus/ros等 | 新增owner绑定的 `context.actions`；小说明安全读取 |
| `core/plugin_registry.py::register/unregister/get` | 可按id找actor | 注册能力/实例generation，卸载先撤销动作 |
| `core/plugin_actor.py::call/cast/_next_invocation` | 同步call，无界cast邮箱 | 动作独立有界邮箱，停止优先槽，短回调预算 |
| `core/astrbot_bridge.py::_execute` | 发布topic即返回published | 新模式接Dispatcher，legacy入口隔离 |
| `core/topic_bus.py` | 满队列丢旧，同步callback | 保持现有语义；不拿它做可靠动作通道 |

新增 `core/actions/models.py/dispatcher.py/ledger.py` 与 `core/decision/catalog.py`。为v2动作插件补明确runtime kind映射；不要求伪装旧motion/skill接口。

## 2. SDK接口与执行约束

```python
def on_action_command(self, command):
    # Validate device readiness, reserve local state, return promptly.
    # Return accepted/rejected; do not wait for a physical action to finish.
    ...

def on_action_cancel(self, command_id, reason):
    # Initiate stop; report canceled only after safe-stop evidence.
    ...

def on_worker_step(self):
    # Bounded work. Advance controller and publish status via actions.report.
    ...

self.context.actions.report(command_id, "running", details={...})
self.context.actions.report(command_id, "succeeded", details={...})
```

接口名在B00冻结。动作handler由插件内固定映射 `action_id → 受信程序`，不从模型传入方法名执行getattr。暂停/恢复只有实现时才声明。上报owner、实例generation由框架补，业务不能伪造其他owner。

同进程Python插件本身不是安全沙箱；这些校验防协议错误和模型越权，不声称防御已安装的恶意Python代码。只加载可信插件。

## 3. 分步施工与验收

| 步骤 | 具体工作 | 本步验收 |
|---|---|---|
| 1 | 实现v2声明校验、动作id唯一性、资源与操作表；保持v1兼容 | 新旧manifest各有独立测试；重复id拒绝 |
| 2 | 加载 `observation_guide`，resolve后限制在插件根内；限大小、UTF-8、纯文本 | `../`、绝对路径、符号链接越界拒绝；缺文档显示unavailable不编造 |
| 3 | Catalog按owner/实例代次收集能力；安装/卸载/启用/配置变化刷新revision | 禁用插件不出现在可执行候选；说明/动作来自同版 |
| 4 | Dispatcher验证goal授权、owner、参数schema、资源、runtime、观察新鲜度 | 任一失败零下发；安全取消不受普通动作白名单阻挡 |
| 5 | Actor新增普通动作队列容量/字节限制，满则返回busy；禁止覆盖已受理指令 | N+1入队明确拒绝，前N条不丢；取消不排在所有普通动作后 |
| 6 | 在受理前登记command_id，原子去重与资源占用；将投递和受理区分 | 重发同id只返回现状；同id不同参数冲突；资源不能双占 |
| 7 | 插件结果直接进入Ledger，合法状态机、event_seq、查询接口 | progress不能覆盖终态；其他owner/旧generation上报被拒 |
| 8 | 加入命令租约、本地超时、取消状态；停止结果不确定标unknown/blocked | 超时后新start关闭；无假canceled；未停资源不释放给新动作 |
| 9 | 增持久执行日志/待发送事件（SQLite即可），与内存状态明确提交边界 | 重启未终结动作变unknown；不自动重发；事件可补查询 |
| 10 | legacy proposal与新模式互斥；所有stop/fault/unload都撤销动作gate | 旧proposal无法绕过新调度；关闭新模式不自动恢复旧运动 |

持久化不能在runtime主锁内做长I/O。持久writer有界；账本/终态事件队列满属于可见故障，应停新动作并保留不确定状态，不能像感知流一样静默丢终态。通知AEB以已提交事件为准。

现有bridge的参数校验不是完整JSON Schema实现，尚未执行additionalProperties、minimum/maximum、长度上限等约束。新接口必须选择固定且受支持的schema子集并完整实现，或使用锁版本的标准校验库；遇到不支持关键字时拒绝声明，不能静默忽略。程序单独拒绝NaN/Infinity、布尔值冒充数字、过深/过大对象；递归校验也要有复杂度上限。

## 4. 取消和安全细节

- 停止优先队列只能抢占未执行的调用，不能杀死卡住的Python方法。测试要故意阻塞handler，证明上层会报故障且下位机租约仍停车。
- 禁止在云请求线程同步等待actor物理动作完成。受理回调建议模拟环境≤20ms；长I/O放worker/子进程，最终预算按设备测量。
- 既有 `runtime.stop()` 末尾把状态置IDLE；需新增动作停止未确认的可见状态，不能继续给新动作放行。是否保留runtime枚举或用decision.blocked表达，由B00确定，不能吞错误。
- `SafetyGuard.filter_intent` 只覆盖旧motion intent路径。新直达动作不能假装自动经过它：全局gate、资源约束和插件本地安全必须显式覆盖；可复用的数值规则通过明确适配调用。
- 每条动作的持续时间/取消时限按声明上界检查。禁用插件/切环境前需要先停动作，不能先卸载后发现没法停车。

## 5. 测试矩阵

新增 `test_action_dispatcher.py`、`test_action_ledger.py`、`test_plugin_action_api.py`、`test_capability_catalog.py`。扩展 `test_plugin_actor.py`、`test_local_plugins_config.py`、`test_astrbot_bridge.py`。

| ID | 场景 | 通过断言 |
|---|---|---|
| A01 | 两owner各100条命令交错 | 零串发，owner只收到自己的命令 |
| A02 | 同id重试10次 | 业务start计数=1；终态可查询 |
| A03 | 满队列/超大参数 | 明确rejected/busy，已受理消息不丢 |
| A04 | 同资源并发start | 同时活动实例≤1；冲突无硬件调用 |
| A05 | 阻塞handler后cancel | 标超时/unknown；不谎报停止；测试看门狗停车 |
| A06 | duplicate/out-of-order终态与progress | 状态不回退，不触发二次任务推进 |
| A07 | 卸载/重载同id插件 | 旧命令和回报不能进入新实例 |
| A08 | 文档路径逃逸/巨大文件 | 安全拒绝；不读插件根外文件 |
| A09 | 进程在受理/执行/提交边界崩溃 | 重启先unknown/停机核对，不重放副作用 |
| A10 | 已fresh的context等待至TTL到期再提交 | 执行时被拒；不能只看缓存fresh |
| A11 | topicbus洪水/慢订阅callback | 动作结果不靠bus到达；安全stop独立可触发 |
| A12 | 额外字段、越限数字、NaN、深层对象、不支持schema关键字 | 全部按契约拒绝，无硬件调用；不能只验证类型就放行 |

通过：新测试100%通过；A01–A12不可skip；1000条随机状态序列无串发/重复start/非法状态；全套现有EX回归无新增失败。真实物理停车不在本批签字。

## 6. 回滚与交接

以 `control_mode=legacy/decision`（命名由B00固定）切换，默认新模式关闭。回滚前先停机核对所有动作；不能卸载新模块后让未停止插件继续运动。交接B03/B04：可运行SDK示例、manifest格式、错误码、状态查询、全部测试输出。
