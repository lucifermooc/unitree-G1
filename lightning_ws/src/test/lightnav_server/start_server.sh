#!/usr/bin/env bash
# Start lightnav-serve (LightNav-0) on this Thor through the repo's scripts/serve_thor.sh.
#
#   bash start_server.sh                    # fp8_llm_only, GPU_MEM_UTIL=0.3, ws://127.0.0.1:8050
#   QUANT=bf16 bash start_server.sh         # bf16 weights instead of fp8 LLM / bf16 ViT
#   HOST=0.0.0.0 bash start_server.sh       # reachable from the LAN -- the server has NO auth
#
# Returns once the server is READY (serve_thor.sh polls up to 10 min); the server keeps
# running in the background. Log: $REPO_DIR/logs/server_$PORT.log. Stop: stop_server.sh.
# Safe to call from a ROS-sourced shell: PYTHONPATH / LD_LIBRARY_PATH are dropped.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"

GPU_MEM_UTIL=${GPU_MEM_UTIL:-0.3}
HOST=${HOST:-127.0.0.1}
QUANT=${QUANT:-fp8}
# The robot stack and the semantic map share the same 122 GB unified memory pool.
MAX_GPU_MEM_UTIL=0.30

if ! awk -v u="$GPU_MEM_UTIL" -v m="$MAX_GPU_MEM_UTIL" 'BEGIN { exit !(u > 0 && u <= m) }'; then
    if [ "${ALLOW_GPU_MEM_UTIL_ABOVE_LIMIT:-0}" != "1" ]; then
        echo "[start_server] GPU_MEM_UTIL=$GPU_MEM_UTIL is outside (0, $MAX_GPU_MEM_UTIL]" >&2
        exit 1
    fi
fi
case "$QUANT" in
    fp8|fp8_llm_only) VQ=fp8_llm_only ;;
    bf16|none)        VQ="" ;;   # serve_thor.sh keeps an explicitly empty VLLM_QUANT = bf16
    *) echo "[start_server] QUANT must be fp8 or bf16, got '$QUANT'" >&2; exit 1 ;;
esac
[ -x "$VENV/bin/python" ] || { echo "[start_server] no venv at $VENV (run install_server.sh)" >&2; exit 1; }
[ -f "$REPO_DIR/scripts/serve_thor.sh" ] || { echo "[start_server] no repo at $REPO_DIR" >&2; exit 1; }
[ -f "$MODEL_PATH/config.json" ] || { echo "[start_server] no checkpoint at $MODEL_PATH" >&2; exit 1; }

# HF_HUB_OFFLINE: the checkpoint is a local directory; never reach for the hub.
exec env -u PYTHONPATH -u LD_LIBRARY_PATH \
    VENV="$VENV" MODEL_PATH="$MODEL_PATH" GPU_MEM_UTIL="$GPU_MEM_UTIL" \
    HOST="$HOST" PORT="$PORT" VLLM_QUANT="$VQ" HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}" \
    bash "$REPO_DIR/scripts/serve_thor.sh"
