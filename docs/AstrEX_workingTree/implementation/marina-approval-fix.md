# MARINA 审批桥接修复（诊断后局部授权）

日期：2026-09-26
范围：`D:\marina\bridge\approval.py`、`D:\marina\tests\test_bridge.py`
未触碰：`D:\Code\AstrBotEX`、`mcp-config.json`、全局 CLI 模型/认证/权限、任何进程

## 结论

审批截断导致的连续拒绝已修复：正常审批请求输出额度从 800 提高到 4096，
并显式识别 `finish_reason=length`（及 `max_tokens` / `content_filter`）为
fail-closed 拒绝，返回可诊断原因，不从 `reasoning_content` 猜审批，也不会因
`content` 为空而自动批准。

## 改动文件

### `bridge/approval.py`（153 行）

1. **输出额度**：`MAX_OUTPUT_TOKENS = 4096`，请求体 `max_tokens` 由 800 改为该值。
   仍为极简 JSON（`response_format=json_object`、`temperature=0`），未改拒绝策略。
2. **超时**：`REQUEST_TIMEOUT_SECONDS = 120`，替换原 `timeout=30`，与更大输出额度匹配。
   无自动批准、无重试逻辑。
3. **截断识别**：新增 `INCOMPLETE_FINISH_REASONS = ("length","max_tokens","content_filter")`。
   新增 `parse_response(body)` classmethod 统一解析：
   - `finish_reason in ("length","max_tokens")` → 抛 `TruncatedApprovalResponse`
     （原因含 `finish_reason=length`，不含 `JSONDecodeError`）；
   - `content_filter` → 抛 `ApprovalResponseError`；
   - `finish_reason != "stop"` → 抛 `ApprovalResponseError`；
   - `content` 非字符串或空白 → 抛 `ApprovalResponseError`（空回复永不批准）；
   - JSON 无效 → 包成 `ApprovalResponseError`，不再泄漏裸 `JSONDecodeError`。
4. **异常层级**：`ApprovalResponseError(RuntimeError)` → `TruncatedApprovalResponse`。
   `review()` 捕获该层级后仅返回本地生成的、无凭证的诊断文本。
5. **保留的安全约束**：`NoRedirect`（不向重定向转发 Bearer key）、HTTP 读取上限
   1 MiB、100000 字符请求上限、审计写入失败即拒绝、`approved` 必须为 JSON 布尔、
   失败/超时一律拒绝。未新增任何放宽权限的改动。

### `tests/test_bridge.py`（768 行）

- `ApprovalHTTPRegressionTests`：真实本地 HTTP mock，覆盖
  - `test_budget_and_timeout_for_reasoning_response`：断言 `max_tokens==4096`、
    `timeout==120`、opener 为 `NoRedirect`；
  - `test_length_empty_reasoning_reply_is_denied_and_audited`：`finish_reason=length`
    + 空 content + reasoning_tokens=800 → 拒绝，原因含 `truncated` 与
    `finish_reason=length`，不含 `JSONDecodeError`，审计记录一致且不含 key；
  - `test_length_with_valid_allow_json_is_still_denied`：截断但 content 是合法
    allow JSON 时仍拒绝；
  - 空/null/空白回复、非法 JSON/非布尔 approved/空 reason、异常 finish、
    畸形 envelope、重定向不转发凭证、审计写失败仍拒绝、并发审计隔离、
    超限请求不发 API 调用。
- `TruncationTests`：`finish_reason=length` 与空 content 均 fail-closed。
- 原有回归（决策解析、启动计划、审批路由、桥接往返、取消、HTTP UTF-8）保持通过。

## 测试命令与结果

```
python -m unittest discover -s tests -p "test_bridge.py"
```

结果：`Ran 54 tests ... OK`（复跑两次一致）。
`python -m py_compile bridge/approval.py tests/test_bridge.py` → OK。

## 重要运行状态

- 正在运行的 MCP Python 进程已载入**旧**模块，文件修复不会自动生效：
  - PID 34548 `bridge\server.py --worker claude ...`，启动于 2026/9/26 15:41:32；
  - PID 15552 `bridge\launch.py --coordinator codex ...`，启动于 2026/9/26 15:41:32。
- 两进程均早于本次修改（approval.py mtime 17:15），因此仍使用 800 额度与旧解析。
- **需由协调端/用户正规重连**（断开并重新下发任务）以载入新模块。
  本次会话**未**做热补丁、未重启或取消任何进程、未直接 shell 启动 worker。

## 未解决 / 需注意

1. **需重连生效**：如上述，修复在旧进程内不生效；生效前审批仍可能截断。
2. **写作并发**：`approval.py` / `test_bridge.py` 在本次会话进行中被另一写入者
   大幅扩展（出现 `parse_response`、`ApprovalResponseError`、批量/并发测试类、
   `probe_approval.py` 等超出本职任务的改动）。最终状态满足全部验收要求且测试
   通过，但这些外部改动**未经本次执行端编写**，其来源与意图需协调端核实。
3. **审批请求体积**：本次会话中多次 Edit 因 diff 较大触发审批截断而被拒
   （即本缺陷本身）。较小改动可稳定通过。重连后应恢复正常。
4. 未提交、未推送；未改 `mcp-config.json`；未放宽任何权限。
