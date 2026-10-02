**REVISED DOCUMENT DRAFT — NOT APPLIED / NOT TESTED**

```markdown
# B00 Decision / Goal Contract

**Status:** frozen protocol and integration design only. This document does not
claim that the EX runtime, AEB, action SDK, HTTP API, SQLite recovery, watchdog,
hardware integration, or any referenced scenario has been implemented or run.

## 1. Transport and method boundary

Business payloads use integer `schema_version: 1` inside the unchanged
`astrbotex-zmq` envelope v1. Unknown methods return structured
`unsupported_method`; they never fall back to the legacy proposal path.
`method` must be a bounded non-empty string. Lists, objects, null, empty strings,
and overlong values return a structured contract error, never a Python
`TypeError`.

| Method | Direction | Request | Result |
|---|---|---|---|
| `decision.context.get` | AEB -> EX | `{schema_version, ex_session?}` | read-only context |
| `decision.goal.submit` | AEB -> EX | `GoalSubmit` | submit admission result |
| `decision.goal.cancel` | AEB -> EX | `GoalCancel` | cancel admission result |
| `decision.goal.renew` | AEB -> EX | `GoalRenew` | lease renewal result |
| `decision.state.get` | AEB -> EX | `{schema_version, ex_session?}` | `DecisionState` |
| `decision.events.get` | AEB -> EX | `EventsRequest` | `EventsReply` |
| `decision.feedback` | EX -> AEB | `Feedback` | durable feedback acknowledgement |

`decision.context.get` and `decision.state.get` are bootstrap reads:
`ex_session` may be omitted on the first connection. If present, it is strictly
validated. Goal control and event requests require `ex_session`. Payload
identity never overrides trusted connection or plugin identity.

All message parsers reject unknown fields, malformed nested structures,
non-finite numbers, cycles, excessive depth/nodes/serialized bytes, and invalid
bounded IDs. `schema_version` must be exactly integer `1`; bool and float values
are rejected. Parsed mutable JSON values are copied at input boundaries and
`to_dict()` returns copies.

## 2. Frozen validation limits

The following limits apply to all decision/action values unless a narrower
field limit is stated:

| Limit | Value |
|---|---:|
| maximum validation depth | 32 |
| maximum traversed nodes | 20,000 |
| maximum UTF-8 JSON bytes | 1,048,576 |
| maximum JSON integer digits | 4,096 |
| maximum ID length | 256 characters |
| maximum goal text length | 4,096 characters |
| JSON integer sequence range | `0..2**53-1` |
| lease range | `1..600000` milliseconds |
| actual event sequence | `>=1` |
| cursor/latest-empty sequence | `0` permitted |

A cycle, non-string object key, serialization failure, or budget violation is a
contract rejection. Validation must occur before code reads nested fields.
Non-ASCII text may produce an advisory language marker, but ASCII is never an
execution safety gate.

Action schemas use a fixed supported subset rather than full JSON Schema:

- `type`: `object`, `array`, `string`, `number`, `integer`, `boolean`, `null`
- `properties`
- `required`
- `additionalProperties` as bool or nested schema
- `enum`
- `items`
- `minLength`, `maxLength`
- `minItems`, `maxItems`
- `minimum`, `maximum`, `exclusiveMinimum`, `exclusiveMaximum`, `multipleOf`
- `format: uuid`
- bounded literal-safe `pattern`

`pattern` is limited to a bounded literal-safe expression: maximum 256
characters, no backreferences, lookaround, recursion, embedded code, or
unbounded/full-regex constructs. Unsupported JSON Schema keywords are rejected,
not ignored.

## 3. Goal messages and revisions

`GoalSubmit` contains:

- `schema_version`
- `request_id`
- `ex_session`
- `task_id`
- `step_id`
- `goal_id`
- `goal_text_en`
- `allowed_actions`
- `parameters`
- `completion`
- positive `lease_ms`
- optional `expected_revision`

`goal_text_en` and `parameters` form one atomic goal revision. Parameter keys
must be action IDs in `allowed_actions`; each parameter value is an object.
`completion.required_success_actions`, when present, must be a subset of
`allowed_actions`. Unknown completion fields are rejected.

`GoalCancel` contains:

- `schema_version`
- `request_id`
- `ex_session`
- `goal_id`
- non-negative bounded `goal_revision`
- optional string `reason_code`

`GoalRenew` contains the same request/session/goal identity and positive
`lease_ms`. Renewal never creates a goal and never changes the goal revision.

A submit result is:

```json
{
  "ok": true,
  "request_id": "req-1",
  "ex_session": "ex-1",
  "goal_id": "goal-1",
  "revision": 9,
  "phase": "accepted"
}
```

An optional structured error is:

```json
{
  "code": "revision_conflict",
  "path": "expected_revision",
  "message": "expected_revision 8 does not match current 9"
}
```

The revision guard is compare-and-advance. A matching `expected_revision`
returns `current + 1`. An omitted `expected_revision` also allocates
`current + 1`; omitting CAS is not permission to reuse the current revision.
The decision module imports the canonical `check_revision` from
`actions.models` and must not define a local shadow implementation. The shared
action helper still requires A's integration patch for the omitted-CAS rule.

The four facts below are distinct:

- `transport_ack`: the envelope was received.
- `submit_accepted`: EX accepted the goal request for processing.
- `goal_active`: the goal passed replacement and gate rules.
- `action_completed`: execution reached a terminal result.

No ACK or admission response implies activation or completion.

## 4. Goal phases and action statuses

Goal phases are:

- `accepted`
- `pending_cancel`
- `active`
- `blocked`
- `rejected`

Only `active` enters decision execution. A replacement remains pending or
blocked until the old action has positive safe-stop evidence. If stopping is
uncertain, the phase is `blocked` and the decision gate remains closed. The
system must not silently switch to the new goal.

Action progression is:

```text
admitted -> accepted -> running -> terminal
```

Terminal statuses are:

- `rejected`
- `succeeded`
- `failed`
- `canceled`
- `timed_out`
- `unknown`

Terminal statuses do not regress. `canceled` requires positive plugin stop
evidence. `unknown` and `timed_out` do not release uncertain resources or
pretend that a stop occurred.

## 5. State, events, and feedback

`DecisionState` contains:

- `schema_version`
- `ex_session`
- non-negative bounded `revision`
- paired optional `active_goal_id` / `active_phase`
- paired optional `pending_goal_id` / `pending_phase`
- object `execution`
- non-negative bounded `event_seq`

An active goal must have phase `active`. A pending goal may have phase
`accepted`, `pending_cancel`, or `blocked`. An ID without its phase, or a phase
without its ID, is invalid.

`EventsRequest` contains:

- `schema_version`
- required `ex_session`
- non-negative bounded `since_event_seq`

`EventsReply` contains required:

- `schema_version`
- `ex_session`
- `events`
- `oldest_available_seq`
- `latest_event_seq`
- boolean `resync_required`

The complete buffered page is validated before any event field is read,
including when resync will discard it. This includes list/type, budget, cycle,
depth, finite-number, nested object, required-field, session, status, and
sequence checks.

Each actual event has positive `event_seq`. Events in one page must be
strictly increasing and unique. Every event must have the same `ex_session` as
the outer reply. Every event sequence must be within the declared
`oldest_available_seq..latest_event_seq` range. An event below
`oldest_available_seq` is invalid even when `resync_required` will be true.

If:

```text
since_event_seq < oldest_available_seq - 1
```

the reply sets `resync_required: true` and contains an empty `events` array.
A reply with `resync_required: true` and events is invalid. If history has been
trimmed and the buffer is empty, the producer supplies an explicit
`latest_event_seq`; it must not be inferred as zero.

`Feedback` contains:

- `schema_version`
- `ex_session`
- `task_id`
- `goal_id`
- non-negative bounded `goal_revision`
- positive bounded `event_seq`
- strict `status`
- optional string `reason_code`
- object `details`

Feedback status is exactly one of:

```text
admitted, rejected, accepted, running, succeeded,
failed, canceled, timed_out, unknown
```

`event_seq: 0` is valid only for cursors or an empty/latest boundary; it is not
valid for an actual feedback event. Persisted feedback acknowledgement means
durable persistence only, not execution completion.

## 6. Action SDK and ownership

The exact callback surface is:

```python
on_action_command(command) -> {
    "accepted": bool,
    "reason_code": str,
    "details": object,
}

on_action_cancel(command_id, reason) -> {
    "accepted": bool,
    "reason_code": str,
    "details": object,
}

actions.report(command_id, status, reason_code='', details=None)
```

`owner` and `generation` are framework-bound by the registered action-owner
slot and trusted plugin generation. Callback payloads cannot claim ownership or
generation. The capability is:

```text
capability: action_owner
runtime_kind: action
```

It is not `motion_bridge`.

The simulated callback budget is 20 ms. Framework-stamped callback context
includes a monotonic deadline, environment generation, plugin generation, and
resource token. These values are internal framework values and never come from
the model or arbitrary command parameters.

`actions.report` is the structured execution/event path. A private
`emit_user_message` path may report user-visible progress/status but is never a
command path. Completion is represented by structured status/events and has no
required text finish.

`keep` means maintain the existing action only. It does not mean start, retain
an arbitrary snapshot/result, or create a new action. No arbitrary handler name,
topic, or executable parameter map is accepted. Start/cancel/pause/resume/keep/
wait/replan semantics must be declared or represented by structured status.
Cloud timeout cannot extend a lease or bypass absolute `max_duration_ms`.

## 7. Action command and event examples

A valid action command contains all command fields:

```json
{
  "schema_version": 1,
  "command_id": "cmd-21",
  "ex_session": "ex-boot-3",
  "goal_id": "goal-9",
  "goal_revision": 9,
  "decision_id": "dec-12",
  "owner": "new_arm",
  "plugin_generation": 3,
  "action_id": "new_arm.pick_selected.v1",
  "operation": "start",
  "params": {
    "target": {
      "observation_id": "obs-40",
      "object_id": "track-8",
      "selected": true
    }
  },
  "lease_ms": 1000
}
```

A valid canceled action event includes positive stop evidence:

```json
{
  "event_id": "evt-22",
  "event_seq": 22,
  "ex_session": "ex-boot-3",
  "task_id": "task-4",
  "goal_id": "goal-9",
  "goal_revision": 9,
  "command_id": "cmd-21",
  "owner": "new_arm",
  "status": "canceled",
  "reason_code": "operator_cancel",
  "details": {},
  "stop_evidence": {
    "stopped": true
  }
}
```

`stop_evidence.stopped` is a strict boolean and must be true for
`canceled`. `unknown` and `timed_out` do not release uncertain resources.

## 8. Observation contract

The manifest interface `observation_sources` is a map:

```json
{
  "front_clearance": {
    "topic": "front_sensor.clearance",
    "max_age_ms": 200,
    "required_fields": ["distance_m", "valid"]
  }
}
```

The topic is an internal TopicBus topic. `front_sensor.clearance` is the
representative internal topic; ROS topic names such as
`/sensors/front_clearance` require an explicit adapter and are not implied by
this contract.

The frozen type is:

```text
observation_sources:
  map<string, {
    topic: string,
    max_age_ms: bounded positive integer,
    required_fields: list<string>
  }>
```

This is the explicit A/B02 type-integration interface.

The observation wrapper is:

```json
{
  "schema_version": 1,
  "observation_id": "obs-40",
  "source_id": "front_clearance",
  "source_epoch": "sensor-boot-3",
  "seq": 40,
  "received_monotonic_ns": 1234567890,
  "age_ms": 37,
  "description_hash": "sha256:...",
  "health": {
    "status": "ok",
    "reason_code": ""
  },
  "data": {
    "distance_m": 0.84,
    "valid": true
  }
}
```

Required wrapper fields are `schema_version`, `observation_id`, `source_id`,
`source_epoch`, `seq`, `received_monotonic_ns`, `age_ms`,
`description_hash`, `health`, and `data`. `source_epoch` is a bounded string,
for example `"sensor-boot-3"`, not an integer.

`health.status` is exactly `ok`, `stale`, or `error`; `reason_code` is a
string. `data` is original domain data and its fields are not rewritten into a
universal raw-data schema. `age_ms` is recomputed at admission from the
monotonic receive timestamp; a caller-provided stale age is not trusted.
Target references are opaque observation/object/track/frame references and do
not impose one payload schema on every sensor.

## 9. Coordinator-frozen snapshot and backend contracts

These shapes are frozen here; B04 implements the backend engine.

### 9.1 VersionSet

```json
{
  "ex_session": "ex-boot-3",
  "goal_revision": 9,
  "config_revision": 12,
  "catalog_revision": 5,
  "environment_generation": "env-7",
  "gate_epoch": 44,
  "plugin_generations": {
    "new_arm": 3,
    "new_base": 2
  }
}
```

`VersionSet` is:

```text
{
  ex_session: string,
  goal_revision: integer,
  config_revision: integer,
  catalog_revision: integer,
  environment_generation: string,
  gate_epoch: integer,
  plugin_generations: map<owner, non-negative integer>
}
```

### 9.2 Snapshot

```json
{
  "schema_version": 1,
  "snapshot_id": "snap-18",
  "created_monotonic_ns": 1234567890,
  "versions": {
    "ex_session": "ex-boot-3",
    "goal_revision": 9,
    "config_revision": 12,
    "catalog_revision": 5,
    "environment_generation": "env-7",
    "gate_epoch": 44,
    "plugin_generations": {
      "new_arm": 3
    }
  },
  "goal": {
    "task_id": "task-4",
    "goal_id": "goal-9",
    "goal_text_en": "Pick the selected cup.",
    "parameters": {
      "new_arm.pick_selected.v1": {
        "target": {
          "observation_id": "obs-40",
          "object_id": "track-8"
        }
      }
    }
  },
  "observations": [],
  "owners": [
    {
      "owner": "new_arm",
      "plugin_generation": 3,
      "status": "ready",
      "candidates": [
        {
          "option_id": "opt-1",
          "kind": "start",
          "action_id": "new_arm.pick_selected.v1",
          "command_id": null,
          "description": "Pick the selected target",
          "eligible": true,
          "reason_code": ""
        }
      ]
    }
  ]
}
```

`Snapshot` is:

```text
{
  schema_version: 1,
  snapshot_id: string,
  created_monotonic_ns: integer,
  versions: VersionSet,
  goal: {
    task_id: string,
    goal_id: string,
    goal_text_en: string,
    parameters: object
  },
  observations: list<observation wrapper>,
  owners: list<{
    owner: string,
    plugin_generation: integer,
    status: string,
    candidates: list<{
      option_id: string,
      kind: start|cancel|pause|resume|keep|wait|request_replan,
      action_id?: string,
      command_id?: string,
      description: string,
      eligible: boolean,
      reason_code: string
    }>
  }>
}
```

Only `start` uses the bound goal parameters. `cancel` and `keep` target an
existing command. `wait` and `request_replan` do not select a handler or create
a new goal. `keep` means maintain the existing action only.

### 9.3 BackendDecision

```json
{
  "schema_version": 1,
  "snapshot_id": "snap-18",
  "versions": {
    "ex_session": "ex-boot-3",
    "goal_revision": 9,
    "config_revision": 12,
    "catalog_revision": 5,
    "environment_generation": "env-7",
    "gate_epoch": 44,
    "plugin_generations": {
      "new_arm": 3
    }
  },
  "backend": "local-policy",
  "model": "policy-v2",
  "elapsed_ms": 14,
  "choices": [
    {
      "owner": "new_arm",
      "option_id": "opt-1",
      "confidence": 0.91,
      "probabilities": {
        "start": 0.91,
        "wait": 0.09
      }
    }
  ]
}
```

`BackendDecision` is:

```text
{
  schema_version: 1,
  snapshot_id: string,
  versions: VersionSet,
  backend: string,
  model: string,
  elapsed_ms: bounded non-negative integer,
  choices: list<{
    owner: string,
    option_id: string,
    confidence?: finite number in [0,1],
    probabilities?: map<string, finite number in [0,1]>
  }>
}
```

`versions` are EX request-ticket metadata, not model-granted authority.
Selected `owner` and `option_id` must exist in the snapshot. No backend result
may introduce a new goal, new parameters, arbitrary handler, new topic, or
unlisted action. Any stale version in the result discards it before admission.
Cloud timeout cannot renew a lease or bypass absolute action duration.

## 10. Ownership, locks, queues, and SQLite

Ownership is divided as follows:

- **Goal coordinator:** goal revisions, replacement phases, and goal gates.
- **Dispatcher:** command admission, version binding, operation checks, and gate
  transitions.
- **Ledger:** command, event, idempotency, and ownership records.
- **Actor queues:** bounded per-action callback delivery.
- **SQLite writer:** serialized durable writes and commit ordering.

All queues are bounded. Locks protect short state transitions only. No runtime
lock may wait for SQLite, a cloud/LLM call, an actor callback, a physical stop,
or hardware. A lock or lease is an ownership primitive, not evidence that a
physical stop succeeded.

SQLite replay is limited to `COMMITTED_FEEDBACK` records that may be
retransmitted to AEB. Physical commands, especially `start`, are never replayed.
After restart, unfinished commands become `unknown`; resources and physical
ownership remain blocked until review. AEB performs `resume_review`. No
automatic physical restart occurs. `ex_session` invalidates stale commands and
replies.

## 11. Control modes and timing

The B08 `mode` resource is exactly:

```text
disabled | shadow | execute
```

It is not a `legacy|decision` enum. `disabled` performs no decision execution;
`shadow` may calculate/report candidate decisions without dispatching physical
commands; `execute` permits admission subject to all gates and versions.

A separate transport compatibility setting, when present, is
`control_mode=legacy` by default. It is orthogonal to `mode`; it does not
replace the `disabled|shadow|execute` decision-mode contract. Legacy and
decision payloads are isolated and never silently translated.

If a stop is uncertain, decision state remains `blocked` and the action gate
stays closed.

Simulation/test budgets are:

- callback handler: 20 ms
- gate closure target: 100 ms
- simulated watchdog lease: 1000 ms
- test timing tolerance: 200 ms

These are software simulation/test budgets only, not hardware braking,
stopping-time, or physical safety guarantees.

## 12. B08 decision management API

The exact twelve B08 paths are:

| # | Method and path | Contract |
|---:|---|---|
| 1 | `GET /api/v1/ex/decision/status` | Read-only service, mode, gate, lease, session, and health status; `200`. |
| 2 | `GET /api/v1/ex/decision/backends` | Read-only backend IDs, versions, capabilities, and availability; `200`. |
| 3 | `GET /api/v1/ex/decision/config` | Redacted non-secret configuration and revisions; `200`. |
| 4 | `POST /api/v1/ex/decision/config` | Configuration mutation with `expected_revision` and `ex_session`; async `202` plus `operation_id`, or conflict `409`. |
| 5 | `POST /api/v1/ex/decision/secret` | Secret set/rotate operation; async `202` plus `operation_id`, or `409`; plaintext is never returned. |
| 6 | `POST /api/v1/ex/decision/test` | Explicit backend/connection test; async `202` plus `operation_id`, or `409`; no physical action. |
| 7 | `POST /api/v1/ex/decision/mode` | Set `disabled`, `shadow`, or `execute` with `expected_revision` and `ex_session`; async `202` plus `operation_id`, or `409`. |
| 8 | `GET /api/v1/ex/decision/catalog` | Read-only action, owner, observation, and version catalog; `200`. |
| 9 | `GET /api/v1/ex/decision/snapshot` | Read-only current coordinator snapshot or explicit stale/not-ready result; `200` when available. |
| 10 | `GET /api/v1/ex/decision/actions` | Read-only admitted/running/terminal action records and structured status; `200`. |
| 11 | `GET /api/v1/ex/decision/decisions` | Read-only backend decision records, selected IDs, versions, and structured reasons; `200`. |
| 12 | `POST /api/v1/ex/decision/stop` | Explicit stop request; async `202` plus `operation_id`, or conflict `409`; completion requires stop evidence. |

Configuration and mode mutations are bound to both `expected_revision` and
`ex_session`. Stale values return `409`. Accepted asynchronous work returns
`202` and does not mean active or complete.

Secret configuration status is visible only through redacted GET responses.
Secret plaintext is never returned by GET, SSE, action results, or backups.
Empty secret input is rejected or treated as no-op; it never clears an existing
secret. Secret save, secret test, configuration save, backend test, and mode
enablement are separate operations and permissions. A successful test does not
save or enable; a save does not enable; enabling does not reveal the secret.

`GET /api/v1/ex/decision/decisions` may expose a bounded, explicitly supplied
read-only prompt for controlled inspection. It returns only structured selected
IDs, versions, statuses, and reasons. It never returns hidden reasoning,
chain-of-thought, secret material, or arbitrary model text. If an SSE
representation is used for decision changes, it emits only `decision_changed`
event IDs and structured references; it does not stream prompts, hidden
reasoning, credentials, or raw backend deliberation.

Management authentication defaults to loopback-only testing. Remote management
requires an explicit management bearer token behind a TLS proxy on an isolated
management port. Cross-origin abuse and forged identity headers are rejected;
identity comes from the trusted connection, not a request header. Credentials
are stored only in secret storage or browser memory. GET, SSE, and backup
storage never contain API keys or bearer tokens.

The existing port `8765` is not certified public-safe by this contract.

## 13. Scenario references

Reports must identify measured scenarios rather than claim execution:

- B02: `A01` through `A12`
- B04: `G01` through `G10`
- B12: `E01` through `E12`

These references cover parser budgets, callback budgets, gate closure, watchdog
tolerance, event replay/resync, restart review, resource blocking, ownership
recovery, backend version staleness, and HTTP `202/409` CAS behavior. Hardware
access, real provider calls, and external API tests are outside this contract
freeze.
```
