#!/usr/bin/env bash
# Domain 63 direct-graph preflight is mandatory in isaac_entry.py. It only
# excludes the verified local ros2cli diagnostic daemon with safe endpoints.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/isaac_common.sh"
exec python -B "$ASTREX_ROOT/scripts/lib/isaac_entry.py" ros "$@"
