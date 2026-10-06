# shellcheck shell=bash
# Shared settings for the LightNav-0 server scripts on the G1 Jetson Thor.
# Sourced by install_server.sh / start_server.sh / stop_server.sh -- not executed.
#
# Every write these scripts make lands under $LN_ROOT or $MODEL_PATH; $HOME/.cache,
# $HOME/.local and the system Python are never touched. Override any value by
# exporting it before calling a script.

: "${LN_ROOT:=/home/unitree/lightnav}"
: "${VENV:=$LN_ROOT/.venv}"
: "${REPO_DIR:=$LN_ROOT/repo}"
: "${MODEL_PATH:=/home/unitree/models/LightNav-0}"
# Not plain `python3`: interactive shells on this Thor resolve that to miniconda 3.14.
: "${PY_BASE:=/usr/bin/python3.12}"
: "${PORT:=8050}"
export LN_ROOT VENV REPO_DIR MODEL_PATH PY_BASE PORT

# Caches that would otherwise default to $HOME/.cache, ~/.triton, ~/.nv, ~/.config.
export PIP_CACHE_DIR="$LN_ROOT/.cache/pip"
export XDG_CACHE_HOME="$LN_ROOT/.cache"
export HF_HOME="$LN_ROOT/.cache/huggingface"
export TORCH_HOME="$LN_ROOT/.cache/torch"
export VLLM_CACHE_ROOT="$LN_ROOT/.cache/vllm"
export VLLM_CONFIG_ROOT="$LN_ROOT/.config/vllm"
export TRITON_HOME="$LN_ROOT/.cache/triton_home"
export TRITON_CACHE_DIR="$LN_ROOT/.cache/triton"
export CUDA_CACHE_PATH="$LN_ROOT/.cache/nv_compute"
export FLASHINFER_WORKSPACE_BASE="$LN_ROOT"
# Triton 3.6's bundled ptxas / ptxas-blackwell are CUDA 12.8 / 12.9 and reject
# sm_110a (Thor is "sm_110" only from CUDA 13 on), so every Triton kernel -- vLLM's
# rotary embedding, torch.compile output -- fails to assemble. Point Triton at the
# CUDA 13.0 ptxas from the nvidia-cuda-nvcc wheel installed into the venv.
export TRITON_PTXAS_BLACKWELL_PATH="$VENV/lib/python3.12/site-packages/nvidia/cu13/bin/ptxas"
# The venv already ignores ~/.local; make it explicit for any helper interpreter.
export PYTHONNOUSERSITE=1
# A ROS-sourced shell puts /opt/ros/... on PYTHONPATH ahead of the venv.
unset PYTHONPATH
