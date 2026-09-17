# AstrEX

AstrEX is an internal research project for embodied task scheduling and execution.

## Repository layout

- `apps/` contains the AstrEX application layer.
- `ros2_ws/` contains the ROS 2 packages.
- `sim/` contains the Isaac Sim integration files.
- `config/` contains the project configuration.
- `scripts/` contains the development scripts.
- `docs/` contains the technical documentation.
- `runtime/` contains local runtime output.

## Isaac baseline

The current AstrEX Isaac development baseline is `isaaclab232_test` (Isaac Sim 5.1.0.0,
Isaac Lab v2.3.2, Python 3.11, Torch 2.7.0+cu128, ROS 2 Jazzy, domain 63). Version
expectations live in `config/isaac_baseline.env`; the baseline, its limits and the evidence
are documented in [docs/ISAAC_51_DEV_BASELINE.md](docs/ISAAC_51_DEV_BASELINE.md).

Three entry points only. Earlier environment reports are historical.

```bash
./scripts/check_isaac_env.sh
./scripts/start_isaac_rl.sh
./scripts/start_isaac_ros.sh
```

Implementation layout: the entry wrappers live in `scripts/`, their private helpers in
`scripts/lib/`, the Isaac scene/OmniGraph/physics code in `sim/scripts/`, and the
regression tests in `tests/isaac/`.

## External data

Large data and generated output are outside this repository:

```text
/data/shared/AstrEX_project_data
```

## Third-party software

`apps/AstrBot` is a Git submodule. AstrBot retains its upstream license.

See `THIRD_PARTY_NOTICES.md` for other external dependencies.

## License

This repository is currently private. No public open-source license is granted.
