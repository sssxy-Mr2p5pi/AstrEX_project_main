# B08 冻结 API v1（实施前，2026-10-01）
所有 /api/ 路径需 Authorization: Bearer；Host 是合法回环域名，Origin 存在时同源。凭据从 data_root/secrets/admin.token 本地读取，HTTP不返回。
管理前缀 /api/v1/ex/decision。标准响应 {ok,ex_session,revision,effective_revision,framework_config_revision,...}；错误 {ok:false,code,message,...versions}，固定错误文本不返回密钥/原始异常。
GET status: cache service/decision/operations summary; backends:真实Mock/Jev/Laya能力; config:saved,effective,secrets状态; catalog:catalog+资格; snapshot:current_goal,last_built,last_submitted,last_result; decisions:items/next_cursor/truncation; actions:items/next_cursor来自Ledger; operations/{id}:operation。
POST body 严格JSON，max64KiB，所有写需 ex_session+expected_revision，stop仅session且忽略旧revision（紧急停止不因旧配置拒绝）。
/config {config:完整已返回saved} ->200新revision，仅保存；/secret {action:set|keep|clear,value:set-only} ->200新revision/keep不改；/test {} ->202独立probe；/mode {mode:disabled|shadow|execute} ->202，显式应用saved并设置mode，过程守住disabled/空闲边界，不启动runtime/模型/Goal；disabled仅关门不应用配置。
/stop {reason?:短文本} ->202立即request_stop/cancel localwait，后续异步query匹配proof；/service/start,stop,recover {} ->202 owned进程序列管理。start需decision disabled无Goal，prewarm独立加载预算，结束仍disabled；stop/recover先撤授权/记录匹配StopEvidence，再终止ownedprocess。recover后freshbackendreplace并stopreview，保持disabled；新mode+新Goal必需。
operation {operation_id,state:pending|running|succeeded|failed|blocked|superseded,kind,ex_session,revision,backend,service_generation,result,error_code,created_ns,completed_ns}; active上限8，完成128，oldintent超代次不生效。
Laya固定typed-decisions/bundleSHA/model/port readonly，trusted部署Python/cache/startcommand不在HTTP。Laya允许现有boolenabled/live_http和已验证数值预算，不含allow_test_execution。Jev地址固定，execute false，probe unsupported不付费。Mock保留工厂，只kind选择等现有可验证配置。
latest128 request records仅内存，默认展示64KiB/record，limit默认20最大100；fullactual bytes hash不因redaction/truncation改变，prepared/post_attempted/post_written_to_socket/response_received事实分别记录，socketwrite不代表远端receipt/完成。EXoutcome与Ledger admitted/accepted/running/succeeded分别关联，未匹配不补造。
恢复配置创建新savedrevision、失效旧operations、decision disabled，不恢复Goal/Ledger/执行权/服务隔离。SSE只新decision ID通知，详细鉴权GET。旧页面memorytoken+fetchSSE/blob，无B09页面。
已知B07 replan0/8保留，不改prompt/策略，不把testActor写成机器人成功。
