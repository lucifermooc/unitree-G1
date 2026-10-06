#!/usr/bin/env bash
# Stop the lightnav-serve instance on $PORT (default 8050) and wait until it has exited
# and released its unified memory. Only that port's server is matched.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"

[ -f "$REPO_DIR/scripts/serve_thor.sh" ] || { echo "[stop_server] no repo at $REPO_DIR" >&2; exit 1; }
exec env -u PYTHONPATH PORT="$PORT" bash "$REPO_DIR/scripts/serve_thor.sh" --stop
