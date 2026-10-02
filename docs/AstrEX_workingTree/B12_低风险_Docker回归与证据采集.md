# B12｜低风险｜Docker回归与证据采集

作用：在本机Docker执行已冻结的测试矩阵，收集可审计证据。预计2–4人日。依赖B03–B11。弱agent可执行，不能自行降低标准或修改高风险生产代码。

## 1. 环境与边界

本次只读检查：Windows Python3.12.10，Docker客户端/服务端29.6.2可用。尚未构建下一代镜像，也未运行下一代集成测试。

使用独立项目名 `hzf-decision-test`、独立data volume与端口（建议本机18765），禁止覆盖原8765或现有AstrBot/NapCat容器。测试默认mock backend、模拟插件、无硬件设备映射、无真实Jev密钥。

本批白名单：新增EX `tests/integration/decision/`、`scripts/verify_decision_flow.py`、`compose.decision-test.yml`、固定镜像测试配置、证据目录；不改Dispatcher/GoalManager/权限/模型门槛。脚本发现产品缺陷要转对应高风险批次修复。

## 2. 测试拓扑

```text
本机Docker独立网络
  AstrBot真实宿主 + AEB新版本
      ↕ ZMQ三通道
  EX新版本 + 模拟插件 + Mock/Jev影子后端
      ↕ HTTP
  Playwright/测试驱动 + TTS/NapCat发送spy

另开ROS容器组：EX ROS镜像 ↔ 外部ROS模拟执行节点
```

本机Docker Desktop上的ROS发现先在同一Docker网络验证，不假设Windows能直接参加Linux DDS多播。香橙派Linux宿主机外节点的验证留B13。

## 3. 步骤和验收

| 步骤 | 工作 | 本步完成条件 |
|---|---|---|
| 1 | 记录源码版本、dirty范围、依赖锁、镜像digest、测试data路径 | 证据可确定测了哪个构建 |
| 2 | 创建隔离compose与mock配置 | 无硬件、无生产volume、无生产端口冲突 |
| 3 | EX全套单元/集成，AEB在真实Host镜像导入及通道测试 | 不用纯stub代替宿主兼容 |
| 4 | 运行B04/B05/B06确定性闭环、故障矩阵 | 每场景留goal/command/event关联日志 |
| 5 | UI验收与密钥/隐式输出泄露扫描 | 截图、控制台、网络请求及标记扫描结果 |
| 6 | Linux ROS镜像跑真实ROS测试和模拟控制器 | DDS业务收发有证据，不只graph可见 |
| 7 | 30分钟负载与故障注入，再做资源回收检查 | 队列受限、线程/连接回收，无累积活动动作 |
| 8 | 汇总缺陷和证据，更新99报告副本或对应栏目 | PASS/FAIL/SKIP/NOT_RUN明确，失败不被覆盖 |

## 4. 命令入口

当前EX可直接使用：

```powershell
Set-Location 'D:\Code\AstrBotEX'
python -m unittest discover -s tests -p 'test_*.py'
```

当前AEB已有Dockerfile.test，其基础镜像是 `local/astrbot:4.26.7-isolated`；施工前核验该镜像存在且对应目标Host代码，不能看到tag就认为版本正确：

```powershell
Set-Location 'D:\Code\A.E.B\astrbot_plugin_astrbotex_interaction'
docker image inspect local/astrbot:4.26.7-isolated
docker build -f Dockerfile.test -t local/hzf-aeb-test .
docker run --rm local/hzf-aeb-test
```

以下为本批新增脚本后的目标入口，并非当前已存在：

```powershell
Set-Location 'D:\Code\AstrBotEX'
docker compose -p hzf-decision-test -f compose.decision-test.yml up -d --build
python scripts/verify_decision_flow.py --url http://127.0.0.1:18765 --scenario all
python scripts/verify_decision_ui.py --url http://127.0.0.1:18765
docker compose -p hzf-decision-test -f compose.decision-test.yml logs --no-color
docker compose -p hzf-decision-test -f compose.decision-test.yml down
```

不要使用 `down -v` 删除证据volume；不要用泛化docker prune。网络拉取/构建、真实付费模型调用和权限提升按执行环境要求申请。所有测试脚本必须有超时和cleanup，失败也保留日志。

## 5. 固定端到端矩阵

| ID | 场景 | 通过标准 |
|---|---|---|
| E01 | 用户长任务→三步goal→成功 | 顺序正确，仅一个active goal，无重复动作 |
| E02 | 第二步失败→LLM重规划→新goal | 先安全停止，参数更新，旧未来步骤被替换 |
| E03 | 全程静默执行 | TTS/NapCat发送=0，内部闭环仍完成 |
| E04 | 每步可选自然发言 | 只公开用户文本，内部唯一标记泄露=0 |
| E05 | 用户中途抢占/改目标 | 有权限才允许；旧决策/旧话术不生效 |
| E06 | Jev超时/429/断网/未知选项 | 本地fail-safe，零不合法动作 |
| E07 | 感知洪水/超大JSON/过期bbox | 有界、拒绝过期绑定，不影响stop通路 |
| E08 | 插件停止失败/未回ACK | blocked/unknown可见，不推进下一步 |
| E09 | AEB/EX分别重启、网络重连 | 核对恢复；不自动重放副作用 |
| E10 | 配置/后端/插件/环境切换 | 旧revision结果拒绝，无混合快照 |
| E11 | 决策页刷新/导航/测试连接 | 产生控制副作用=0 |
| E12 | 快照备份/恢复 | 无secret，恢复后不自执行 |

## 6. 通过标准与证据

- 全部确定性新测试100%通过；原有测试无新增失败。Windows真实ROS skip可以存在，但Linux真实ROS目标环境不能拿skip充通过。
- E01–E12每场景至少20次；乱序/重发/旧代次测试另累计1000条序列，越权/重复start/串发/泄密均为0。
- 稳定负载30分钟：队列峰值不超配置上限；记录RSS/线程/连接曲线。建议暖机5分钟后RSS增长≤20%且无持续线性增长；若超阈值转强agent分析，不能靠放宽阈值过关。
- 性能报告分开：观测接收到快照、模型往返、模型返回到命令受理、取消触发到gate关闭、远端实际停止。不能用网络请求耗时冒充机械响应。
- 证据结构建议 `evidence/<run_id>/versions.json, commands.txt, unit.log, scenarios.json, metrics.csv, ui/`。文件由测试脚本或正常工具生成，不能手填“通过”代替日志。

本批基于mock的流程通过不代表Jev语义效果、真TTS、真NapCat或实物机器人通过。相关项明确另列。

## 7. 失败与回滚

失败按owner批次分流：调度B02/B04，任务B05，公开输出B06，模型B07，API B08，UI B09，视觉B10，ROS B11。弱agent可缩小复现，但不能改预期掩盖产品错误。

清理只停止本批compose项目；保留证据与测试卷。核验生产容器ID/端口未变化后交接B13。
