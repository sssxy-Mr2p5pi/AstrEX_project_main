# B00 implementation — checkpoint 1 (2026-09-26)

## Round outcome: INCOMPLETE — MARINA approval outage again

The MARINA approval channel failed for the second time with the exact fault the
handoff describes: `DeepSeek approval unavailable or invalid (JSONDecodeError)`
on a substantial `Edit` payload (`bridge/approval.py` reasoning-token exhaustion
-> empty content -> JSONDecodeError -> fail-closed deny). Per the task
instruction, the round ended after the technical error rather than retrying
other tools, changing permissions, or bypassing approval.

Two edits to `astrbot_ex/core/actions/models.py` were APPROVED and applied
before the denial; the third was DENIED and not applied.

## Baseline reproduced independently (raw, this round)

Command (EX root `D:\Code\AstrBotEX`, HEAD bd6b30b7013fff33fcc002f827ee64f2f5d21eb5):

    PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -p 'test_*.py'

Result: `Ran 137 tests in 5.105s`, `FAILED (failures=1, skipped=5)`, exit 1.
131 original tests unchanged (5 ROS skips); the 6 added tests are the previous
worker's draft `tests/test_decision_contracts.py`, of which 1 fails with 17
failing contract cases:

    S-GOAL-NO-SCHEMA: expected missing_field, got unsupported_schema_version
    S-MF-V2-OK: unexpectedly rejected: missing_field
    S-MF-DUP, S-MF-CANCEL-NO-TIMEOUT, S-MF-TIMEOUT-NO-CANCEL,
    S-MF-RUNTIME-STATE, S-MF-DANGER, S-MF-OPERATION: got missing_field
    S-CMD-OK: unexpectedly rejected: missing_field
    S-CMD-UNKNOWN-ACTION, S-CMD-OWNER, S-CMD-EXTRA-FIELD, S-CMD-ENUM,
    S-CMD-RANGE, S-CMD-CANCEL-UNSUPPORTED, S-CMD-STALE-SESSION,
    S-CMD-BAD-REVISION: got missing_field

Root cause of that cluster (separate from the coordinator findings): the stale
generated `astrbot_ex/core/contracts.py` carries a `validate_params` that
returns early on root `type != "object"` and never runs `_check_schema_supported`
first, so nested schemas report a bogus missing field. The canonical source
(`core/actions/models.py`) already has the corrected `validate_params` +
`check_schema_supported` pair; the generated file was never rebuilt.

## Confirmed defects still present (not yet fixed)

Canonical `astrbot_ex/core/actions/models.py` (and the generated copy):

1. `_validate_node` returns immediately when `type` is absent, so an enum-only
   nested schema (no `type`) accepts out-of-enum values, and object/array enums
   are skipped by the early `return` before the trailing enum check.
2. `_check_numeric` silently ignores a non-numeric keyword value, so
   `maximum: "1"` with value 99 is accepted; no keyword type/bounds/consistency
   validation; unbounded recursion in `_check_schema_supported`/`_validate_node`;
   `maximum: true` is admitted as a bound (bool/number conflation).
3. `IdempotencyRegistry.resolve` stores the caller's mutable payload by
   reference; mutating the original object makes changed content look like a
   replay. `{"a": 1}` and `{"a": True}` compare equal in Python.
4. `schema_version` / `action_api_version` are compared with `!=`, so `True`
   passes as version 1, and `action_api_version: 2.0` is accepted.
   `check_revision(None, current)` returns `current` rather than the next
   revision, and does not reject a non-int `current`.
5. `validate_command_against_manifest` silently skips the session, revision and
   runtime-state gates when the caller passes `None`, yet is documented as
   "full admission".

## Applied this round (approved edits, additive only)

`astrbot_ex/core/actions/models.py`:

- added `import json`
- added ErrorCode constants `VALUE_BUDGET_EXCEEDED`, `SCHEMA_DEPTH_EXCEEDED`,
  `SCHEMA_BOUNDS_CONFLICT`
- added `MAX_VALIDATION_DEPTH = 32`, `MAX_VALIDATION_NODES = 20_000`,
  `MAX_VALIDATION_BYTES = 1_048_576` and `measure_json_budget` / `_budget_error`

Syntax verified: `ast.parse` OK. Behaviour is unchanged so far (no helper is
wired in yet), so the tree is consistent and the 137/1-failure baseline is the
current measure.

## Denied / not applied

- Rewriting `find_non_finite` to be iterative with a visited set and a depth
  bound (part of finding 2 + 4 hardening).

## Next actions (checkpoint for the next round)

1. Wire the budget helpers into `GoalSubmit.parse` / `ActionCommand.parse` /
   `ActionEvent.parse` / `Feedback.parse` / `parse_action_manifest` and replace
   `find_non_finite` with the iterative bounded version.
2. Rewrite the validator core in `core/actions/models.py`: apply `enum` for any
   resolved type and for enum-only schemas; validate schema keyword value types
   and cross-keyword bounds; reject `bool` where a numeric keyword is required.
3. Make `IdempotencyRegistry` store `json.dumps(..., sort_keys=True)` canonical
   form and refuse to overwrite a prior record.
4. Strict version/sequence typing; resolve `check_revision` semantics
   (`None` -> new goal requires a new revision) and document CAS-vs-assign;
   make the `None`-skippable admission preconditions explicit and auditable.
5. Fix `scripts/build_contracts.py` `_strip` to dedupe the `__future__`/
   stdlib import block, then regenerate `contracts.py` and
   `scripts/sync_contract_mirror.py` into
   `D:\Code\A.E.B\astrbot_plugin_astrbotex_interaction\task_contracts.py`.
6. Add fixture cases for each finding (enum-only schema, string `maximum`,
   bool-as-version, mutation-is-not-replay, true-vs-1, admission with explicit
   `None`) and an independent AEB test under the plugin's `tests/` that imports
   only `task_contracts` and the fixture (no real Host import).
7. Write `EX/docs/DECISION-CONTRACT.md`; log raw outputs and update
   `implementation/PROGRESS.md`.

## Rollback

All new files are untracked; nothing was committed or pushed. To drop this
round's two approved edits, revert `astrbot_ex/core/actions/models.py` to its
pre-round content — they are purely additive (one import, three constants, two
functions) and can also be left in place as the starting point for the next
round.

Real models, hardware, ROS and Docker acceptance: **NOT_RUN**.
