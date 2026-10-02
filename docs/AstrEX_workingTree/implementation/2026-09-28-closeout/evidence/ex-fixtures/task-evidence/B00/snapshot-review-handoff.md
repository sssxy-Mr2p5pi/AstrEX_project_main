# B00 snapshot review checkpoint (2026-09-28)

Scope: authored snapshot contract surfaces only: `astrbot_ex/core/decision/models.py`, generated `astrbot_ex/core/contracts.py`, `tests/test_snapshot_contracts.py`, and `docs/DECISION-CONTRACT.md`; new review evidence under this directory. The environment timing test and all prior failed logs were preserved and not edited in this follow-up.

## Fixes

- `validate_backend_selection` now canonical-parses defensive `to_dict()` snapshots of the mutable `DecisionSnapshot`, `BackendDecision`, and trusted `VersionSet` at helper entry. Directly constructed invalid models, post-parse mutations, NaN confidence/elapsed/probabilities, bool generations, duplicate owners, missing owners and version mutations are rejected without Python type errors or caller mutation.
- Selection enforces exactly one choice for every snapshot owner, rejects duplicate owner choices, requires eligible options for the same owner, validates probability keys and sum tolerance, and returns defensive selected copies.
- Nested snapshot parsing rebases observation errors to `observations[index].health.status` and version errors to `versions.plugin_generations.owner` / `versions.environment_generation`, while standalone observation/version parsers retain their local paths.
- Added authored/generated tests for the reproduced duplicate-choice omission (`['arm-start', 'arm-start']`), direct invalid construction, second-observation path, version generation path, authored/generated selection parity, and baseline round-trip.
- Documentation now states helper revalidation and the parser-backed nested error path behavior.

## Verification

- `python scripts/build_contracts.py --check`: passed (`contracts.py is current`).
- `python -m unittest tests.test_snapshot_contracts tests.test_contract_validation tests.test_goal_contract_validation tests.test_decision_contracts -v`: **49 passed**, raw subprocess output `snapshot-review-focused.log`.
- `PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -v`: **181 run, 176 passed, 5 expected ROS skips**, exit 0, raw subprocess output `snapshot-review-full.log`.
- `git diff --check`: passed.
- Local-only generated mirror `snapshot-review-task_contracts.py` is byte-identical to canonical, SHA256 `020311d5c44acf9e448ab1f329b915454c0f9e214f53a411c7238d2479c57afc`; no AEB sibling write was made.

## Remaining issues

AEB mirror/runner parity and coordinator final review remain outstanding. The five ROS tests were skipped by their existing dependency guard. B02/B04 runtime freshness, trusted admission, resource checks, dispatch, and backend integration remain unimplemented by design. This checkpoint does not claim full B00 acceptance.
