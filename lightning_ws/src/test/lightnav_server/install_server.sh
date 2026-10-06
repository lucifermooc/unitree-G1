#!/usr/bin/env bash
# Install the LightNav-0 inference server (lightnav-serve) on a Unitree G1 Jetson Thor
# (JetPack 7.0 / R38.2.1, sm_110, Python 3.12). Run ON the Thor from a clean, non-ROS
# shell; it is idempotent, so a failed step can simply be re-run:
#
#   bash install_server.sh                          # all steps
#   STEPS="model verify" bash install_server.sh     # only some steps
#   nohup bash install_server.sh > /home/unitree/lightnav/logs/install.log 2>&1 &
#
# Steps: venv repo torch vllm deps model verify
# Writes only under $LN_ROOT and $MODEL_PATH (see env.sh). No sudo, no ~/.local.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"

# ---------------------------------------------------------------- versions ----
# torch: the official cu130 build. It carries sm_110 SASS and pulls the CUDA 13
# runtime as nvidia-* wheels; this Thor has no system CUDA toolkit (no libcudart /
# cuBLAS / cuDNN under /usr), which the Jetson AI Lab torch build links against.
TORCH_VER=${TORCH_VER:-2.10.0+cu130}
TORCHVISION_VER=${TORCHVISION_VER:-0.25.0+cu130}
# vLLM: Jetson AI Lab build, compiled natively for sm_110. The official 0.19.1
# aarch64 wheel has no sm_110 SASS. Its Python sources for everything lightnav
# patches are identical to 0.19.1.
VLLM_VER=${VLLM_VER:-0.19.0+cu130}
CUTLASS_DSL_VER=${CUTLASS_DSL_VER:-4.5.2}      # must stay pinned: newer dev builds drop ThrMma
TRANSFORMERS_VER=${TRANSFORMERS_VER:-5.8.0}    # lightnav's exact pin
# compressed-tensors 0.14.0.1 (vllm 0.19.0's pin) requires transformers<5; 0.15.0.1
# is what vllm 0.19.1 pins next to transformers 5.x.
COMPRESSED_TENSORS_VER=${COMPRESSED_TENSORS_VER:-0.15.0.1}
# Only for its CUDA 13.0 ptxas (sm_110a); env.sh points TRITON_PTXAS_BLACKWELL_PATH at it.
# nvvm / crt are what pip resolved for it on 2026-10-03.
NVCC_VER=${NVCC_VER:-13.0.88}
NVVM_VER=${NVVM_VER:-13.4.92}
PIP_VER=${PIP_VER:-25.3}

# ----------------------------------------------------------------- sources ----
INDEX_PYPI=${INDEX_PYPI:-https://mirrors.aliyun.com/pypi/simple}
INDEX_JAL=${INDEX_JAL:-https://pypi.jetson-ai-lab.io/sbsa/cu130/+simple}
INDEX_TORCH=${INDEX_TORCH:-https://download.pytorch.org/whl/cu130}
REPO_URL=${REPO_URL:-https://github.com/BoragoCode/LightNav-0}
REPO_REF=${REPO_REF:-c6f40e3220edbf7011e4f17eaf2c865416737d4d}
GIT_TIMEOUT=${GIT_TIMEOUT:-300}
HF_REPO=${HF_REPO:-LightOriginsHQ/LightNav-0}
MODEL_REVISION=${MODEL_REVISION:-826dc5fbfa37afa8293d2e336d329b6ffc0bfb64}
HF_MIRROR=${HF_MIRROR:-https://hf-mirror.com}
# The 9.7 GB weight file: hf-mirror 302s it to cas-bridge.xethub.hf.co, ~30 KB/s
# from this network. ModelScope hosts the identical object (same sha256 as the HF LFS
# oid below) at ~7-9 MB/s. WEIGHTS_FROM=hf uses the mirror instead.
WEIGHTS_FROM=${WEIGHTS_FROM:-modelscope}
WEIGHT_FILE=model-00001-of-00001.safetensors
WEIGHT_URL=${WEIGHT_URL:-https://www.modelscope.cn/models/LightOriginsHQ/LightNav-0/resolve/master/$WEIGHT_FILE}
WEIGHT_SHA256=${WEIGHT_SHA256:-ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18}
WEIGHT_SIZE=${WEIGHT_SIZE:-9695744034}

STEPS=${STEPS:-venv repo torch vllm deps model verify}

PY="$VENV/bin/python"
CONSTRAINTS="$LN_ROOT/constraints.txt"
log() { echo "[install $(date +%T)] $*"; }
pip_install() { "$PY" -m pip install --disable-pip-version-check "$@"; }

[ "$(id -u)" -ne 0 ] || { echo "do not run as root" >&2; exit 1; }
[ -x "$PY_BASE" ] || { echo "PY_BASE not found: $PY_BASE" >&2; exit 1; }
# pip unpacks multi-GB wheels into $TMPDIR; keep that under LN_ROOT as well.
export TMPDIR="$LN_ROOT/.cache/tmp"
mkdir -p "$LN_ROOT/logs" "$LN_ROOT/wheels" "$TMPDIR"

write_constraints() {
    cat >"$CONSTRAINTS" <<EOF
torch==$TORCH_VER
torchvision==$TORCHVISION_VER
nvidia-cutlass-dsl==$CUTLASS_DSL_VER
nvidia-cutlass-dsl-libs-base==$CUTLASS_DSL_VER
EOF
}

# ------------------------------------------------------------------- steps ----
step_venv() {
    # No python3.12-venv (ensurepip) on this image and no sudo: create the venv
    # without pip, then bootstrap pip from its own wheel.
    [ -x "$PY" ] || { log "creating venv $VENV"; "$PY_BASE" -m venv --without-pip "$VENV"; }
    if ! "$PY" -m pip --version >/dev/null 2>&1; then
        local whl="$LN_ROOT/wheels/pip-$PIP_VER-py3-none-any.whl"
        if [ ! -f "$whl" ]; then
            log "fetching pip $PIP_VER from $INDEX_PYPI"
            "$PY" - "$INDEX_PYPI" "$PIP_VER" "$whl" <<'EOF'
import re, sys, urllib.parse, urllib.request
index, ver, out = sys.argv[1:4]
page = index.rstrip("/") + "/pip/"
html = urllib.request.urlopen(page, timeout=60).read().decode()
name = f"pip-{ver}-py3-none-any.whl"
for href in re.findall(r'href="([^"]+)"', html):
    href = href.split("#")[0]
    if href.endswith(name):
        urllib.request.urlretrieve(urllib.parse.urljoin(page, href), out)
        break
else:
    sys.exit(f"{name} not found on {page}")
EOF
        fi
        "$PY" "$whl/pip" install --no-index --disable-pip-version-check "$whl"
    fi
    "$PY" -m pip --version
}

step_repo() {
    if [ -f "$REPO_DIR/pyproject.toml" ]; then
        local head
        head=$(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || echo "?")
        log "repo present at $REPO_DIR (HEAD $head)"
        [ "$head" = "$REPO_REF" ] || [ "$head" = "?" ] || log "WARN: HEAD != REPO_REF $REPO_REF"
        return 0
    fi
    local tmp="$LN_ROOT/.repo_tmp"
    rm -rf "$tmp"
    log "git clone $REPO_URL @ $REPO_REF"
    if timeout "$GIT_TIMEOUT" git clone -q "$REPO_URL" "$tmp" && git -C "$tmp" checkout -q "$REPO_REF"; then
        mv "$tmp" "$REPO_DIR"
    else
        # git-over-HTTPS to GitHub is intermittently unusable here; the codeload
        # tarball of the same commit (no .git) has always worked.
        log "git clone failed; falling back to the codeload tarball"
        rm -rf "$tmp"; mkdir -p "$tmp"
        local tgz="$LN_ROOT/wheels/LightNav-0-$REPO_REF.tar.gz"
        curl -fL --retry 3 -o "$tgz" "https://codeload.github.com/${REPO_URL#https://github.com/}/tar.gz/$REPO_REF"
        tar -xzf "$tgz" -C "$tmp" --strip-components=1
        echo "$REPO_REF" >"$tmp/.lightnav_ref"
        mv "$tmp" "$REPO_DIR"
    fi
    log "repo ready at $REPO_DIR"
}

step_torch() {
    write_constraints
    log "torch $TORCH_VER + torchvision $TORCHVISION_VER (official cu130, sm_110)"
    pip_install --index-url "$INDEX_PYPI" --extra-index-url "$INDEX_TORCH" \
        "torch==$TORCH_VER" "torchvision==$TORCHVISION_VER"
}

step_vllm() {
    write_constraints
    # The JAL wheel does not declare torch, so the torch from step_torch stays; the
    # constraints file makes any attempt to swap it (or cutlass-dsl) a hard error.
    log "vllm $VLLM_VER (Jetson AI Lab, native sm_110) + nvidia-cutlass-dsl $CUTLASS_DSL_VER"
    pip_install -c "$CONSTRAINTS" --index-url "$INDEX_JAL" --extra-index-url "$INDEX_PYPI" \
        "vllm==$VLLM_VER" "nvidia-cutlass-dsl==$CUTLASS_DSL_VER"
}

step_deps() {
    write_constraints
    # Replaces the transformers 4.x that vllm 0.19.0's metadata pulled in; pip prints a
    # resolver warning about that pin, which is expected (see README).
    log "transformers $TRANSFORMERS_VER + lightnav runtime deps"
    pip_install -c "$CONSTRAINTS" --index-url "$INDEX_PYPI" \
        "transformers==$TRANSFORMERS_VER" "tokenizers>=0.22,<0.23" \
        "compressed-tensors==$COMPRESSED_TENSORS_VER" "huggingface_hub>=1.5,<2" \
        "nvidia-cuda-nvcc==$NVCC_VER" "nvidia-nvvm==$NVVM_VER" "nvidia-cuda-crt==$NVVM_VER" \
        "safetensors>=0.7" "numpy>=2.0" "pillow>=11.0" "pyyaml>=6.0" "websockets>=12.0" \
        "opencv-python-headless>=4.10" "imageio>=2.37" "imageio-ffmpeg>=0.6"
}

fetch_weights() {
    local f="$MODEL_PATH/$WEIGHT_FILE"
    if [ "$(stat -c %s "$f" 2>/dev/null || echo 0)" = "$WEIGHT_SIZE" ]; then
        log "$WEIGHT_FILE present ($WEIGHT_SIZE bytes)"; return 0
    fi
    log "downloading $WEIGHT_FILE from $WEIGHT_URL"
    local i have
    for i in $(seq 1 30); do
        have=$(stat -c %s "$f.part" 2>/dev/null || echo 0)
        [ "$have" -ge "$WEIGHT_SIZE" ] && break
        log "  attempt $i: $have / $WEIGHT_SIZE bytes"
        curl -fL -sS -C - --connect-timeout 20 --speed-limit 100000 --speed-time 60 \
            -o "$f.part" "$WEIGHT_URL" || sleep 5
    done
    have=$(stat -c %s "$f.part" 2>/dev/null || echo 0)
    [ "$have" = "$WEIGHT_SIZE" ] || { log "size $have != $WEIGHT_SIZE"; return 1; }
    log "verifying sha256"
    [ "$(sha256sum "$f.part" | cut -d' ' -f1)" = "$WEIGHT_SHA256" ] || { log "sha256 mismatch"; return 1; }
    mv -f "$f.part" "$f"
    log "$WEIGHT_FILE OK"
}

step_model() {
    mkdir -p "$MODEL_PATH"
    local exclude=(--exclude "*.safetensors")
    [ "$WEIGHTS_FROM" = "hf" ] && exclude=()
    log "checkpoint files from $HF_MIRROR ($HF_REPO @ $MODEL_REVISION)"
    HF_ENDPOINT="$HF_MIRROR" HF_HUB_DISABLE_XET=1 HF_HUB_DISABLE_TELEMETRY=1 \
        "$VENV/bin/hf" download "$HF_REPO" --revision "$MODEL_REVISION" \
        --local-dir "$MODEL_PATH" ${exclude[@]+"${exclude[@]}"}
    [ "$WEIGHTS_FROM" = "hf" ] || fetch_weights
}

step_verify() {
    local nv="" d
    for d in "$VENV"/lib/python*/site-packages/nvidia/*/lib; do
        [ -d "$d" ] && nv="${nv:+$nv:}$d"
    done
    # A file, not stdin: triton.jit needs inspect.getsource() on the kernel.
    local chk="$TMPDIR/verify_env.py"
    cat >"$chk" <<'EOF'
import importlib.metadata as md
for p in ("torch", "torchvision", "vllm", "transformers", "tokenizers", "nvidia-cutlass-dsl",
          "compressed-tensors", "flashinfer-python", "triton", "huggingface_hub", "websockets"):
    try:
        print(f"  {p:20s} {md.version(p)}")
    except md.PackageNotFoundError:
        print(f"  {p:20s} MISSING")
import torch
assert torch.cuda.is_available(), "torch.cuda.is_available() is False"
cap = torch.cuda.get_device_capability()
print(f"  device {torch.cuda.get_device_name(0)}  capability {cap}  torch archs {torch.cuda.get_arch_list()}")
assert cap == (11, 0), f"expected sm_110 (Thor), got {cap}"
x = torch.randn(512, 512, device="cuda", dtype=torch.bfloat16)
float((x @ x).float().sum())
import vllm._C, vllm._moe_C  # noqa: F401  compiled vLLM kernels load against this torch
from vllm.platforms import current_platform
print(f"  vllm platform {current_platform.device_name}  capability {current_platform.get_device_capability()}")

import triton
import triton.language as tl


@triton.jit
def _plus_one(x_ptr, y_ptr, n, BLOCK: tl.constexpr):
    i = tl.program_id(0) * BLOCK + tl.arange(0, BLOCK)
    m = i < n
    tl.store(y_ptr + i, tl.load(x_ptr + i, mask=m) + 1.0, mask=m)


a = torch.arange(1000, device="cuda", dtype=torch.float32)
b = torch.empty_like(a)
_plus_one[(4,)](a, b, 1000, BLOCK=256)
assert torch.equal(b, a + 1.0), "triton kernel produced wrong output"
tool = triton.knobs.nvidia.ptxas_blackwell
print(f"  triton kernel on sm_110 OK (ptxas {tool.version}: {tool.path})")

from lightnav.inference.vllm_utils import _assert_vllm_version
_assert_vllm_version()
import lightnav.serving.ws_server  # noqa: F401
print("  torch / vllm / lightnav imports OK")
EOF
    LD_LIBRARY_PATH="$nv${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" PYTHONPATH="$REPO_DIR/src" "$PY" "$chk"
    # Expected, all metadata-only: vllm 0.19.0 pins transformers<5 and
    # compressed-tensors==0.14.0.1; nvidia-cusparselt-cu13 0.8.0's WHEEL file declares
    # "manylinux2014_sbsa", a tag pip does not map to aarch64 (the library loads fine).
    log "pip check (3 expected lines: vllm -> transformers / compressed-tensors, cusparselt platform tag)"
    "$PY" -m pip check || true
    local f
    for f in config.json eval_config.json processor_config.json tokenizer.json tokenizer_config.json \
             chat_template.jinja action_tokenizer/manifest.json "$WEIGHT_FILE"; do
        [ -e "$MODEL_PATH/$f" ] || { log "MISSING $MODEL_PATH/$f"; return 1; }
    done
    log "checkpoint files present in $MODEL_PATH"
}

for s in $STEPS; do
    case "$s" in
        venv) step_venv ;; repo) step_repo ;; torch) step_torch ;; vllm) step_vllm ;;
        deps) step_deps ;; model) step_model ;; verify) step_verify ;;
        *) echo "unknown step: $s (valid: venv repo torch vllm deps model verify)" >&2; exit 2 ;;
    esac
done
log "done: $STEPS"
