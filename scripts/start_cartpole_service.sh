#!/usr/bin/env bash
# Isaac starts first. Its mandatory direct ROS-domain preflight stays unchanged.
# The system Jazzy service starts only after this launcher's own Sim is ready.
set -euo pipefail
ASTREX_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec /usr/bin/python3 -B "$ASTREX_PROJECT_ROOT/scripts/lib/cartpole_service_launcher.py" "$@"
