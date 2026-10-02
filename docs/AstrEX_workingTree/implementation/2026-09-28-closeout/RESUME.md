# 明日接续（当前已按用户要求暂停）

截止 2026-09-28 17:03，所有本会话 Claude 工作进程及已识别子进程已停止。用户明确恢复前，不得继续开发、评测或派新任务。今日正式报告：../../进度报告_2026-09-28.md。

## 不可变的范围

B00–B02 优先，后续继续框架。仅框架、mock、无硬件仿真与工程对接文档；禁止实施 YOLO、底盘、机械臂等工程插件。失败测试不可删除或放宽以提高通过率。继续使用 MARINA Claude MCP 实现、协调端审查；不改变全局权限或绕过审批。

## 权威源码与接续顺序

共同工作区前缀：C:/Users/17088/AstrBotEX-work/hzf-20260927/

1. ex-fixtures 是最新 B00 权威工作区。最新 contracts SHA256 为 020311d5c44acf9e448ab1f329b915454c0f9e214f53a411c7238d2479c57afc。最后 snapshot-review-handoff.md、49 项聚焦和 181 项完整测试日志已保存；协调端还未独立复跑最后返修。重点复核 helper 使用时重新解析、所有 owner 恰好一次、嵌套错误路径及原始输入不变。
2. AEB mirror 仍是旧 84e2e7799ccf73c67e088e6b893b5a3085a7b4b10f7c8eaf937623d0902416c4。需要更新镜像、runner、81 案例 golden，核验源码/生成/两端字节一致并重跑 AEB 实际宿主。不要把旧 26 项宿主通过记录算作最新快照版已验收。
3. ex-content 是 B01 文档、24 场景、字段目录的来源；6 项协调端文件测试通过。同目录还有 B02 SDK/Catalog/LocalPluginManager 扩展，最后 loader 对正常 `<`、`>` 误拒绝已删去，但新增交叉测试和修正后回归未完成。原 24 项 SDK 测试在 Windows/OrangePi 均通过，发生于该最后改动之前。
4. ex-platform 保存已审 Actor 和 registry 基础实现。Actor 独立 26 项+44 子测试通过。registry 有未修复竞态：on_runtime_start 被事件阻塞时并发 stop_runtime，随后放行 start，可能 registry 已停但 actor._runtime_active=True，slot.state=stopping 且 stop callback 未调用；enable 路径也需覆盖。不要合入为安全完成状态。
5. ex-b02 已整合较早 B00 基线、B01、Actor 和账本。账本独立 Windows/OrangePi 各 21 项通过。SDK/registry 最新改动尚未整合进这里，B00 也落后于 ex-fixtures；禁止反向用旧模型覆盖新契约。Dispatcher、运行时/环境撤销、配置重载事务和 A01–A12 未完成。
6. aeb 是独立 AEB 工作树。D:/Code/A.E.B 的用户原有 A.E.B.md、根 task_models.py 和缓存修改必须保留。主仓库尚无本轮集成、提交或部署。

原生 ROS 证据：5 项真实 ROS +21 项账本，26/26 通过无跳过。远端隔离目录 /home/orangepi/hzf-acceptance-20260928-1530；SDK 24/24 在 /home/orangepi/hzf-sdk-review-20260928-1645。禁止动生产目录/服务。未来新版本接线后仍需做对应版本的验证。

## 现场与备份

本目录 uncommitted-source-checkpoint.zip 保存 145 个文件，uncommitted-source-final-delta.zip 覆盖最后变动的 ex-content/astrbot_ex/core/local_plugins.py；配套 uncommitted-source-final-manifest.json 是最终文件哈希。原工作区为首选接续位置，压缩包是备份，不应无条件覆盖工作区。pre-integration 原始脏文件备份在 C:/Users/17088/AstrBotEX-work/hzf-20260928/pre-integration，组件同步备份在 component-integration-1520。

最后关闭了本次 MARINA 会话 f32bcb5ee523416a9b44bb34f1be8ca1 的 9 个 Claude 进程。工作目录保留，旧会话 threadId 不再当作可续接句柄。恢复时读 D:/marina/WORKFLOW.md 和项目索引最小协议，通过新 MCP 任务继续，不直接 shell 启动 worker。



> 2026-10-02：上述两个历史源码备份 ZIP 已迁移至 [/data/shared/AstrEX_project_data/logs/app/ex_pre_b09_20261002/historical-source-backups/](/data/shared/AstrEX_project_data/logs/app/ex_pre_b09_20261002/historical-source-backups/)。它们仅用于历史恢复，不能覆盖当前 B08 基线。
