#!/usr/bin/env bash
# 一键部署：检查环境 → 启动 Qdrant(Docker) → 装 Python 依赖 → 下载模型 → 建库 → 测试查询
# 用法（在 semantic_map/ 目录下）：  bash setup.sh
# 可以重复运行，已经完成的步骤会跳过或很快完成。
set -euo pipefail
cd "$(dirname "$0")"

step() { echo; echo "========== $1 =========="; }
fail() { echo "❌ $1"; exit 1; }

step "1/6 检查 Docker 和 Python"
command -v docker >/dev/null || fail "没装 Docker。安装：https://docs.docker.com/engine/install/ubuntu/"
docker info >/dev/null 2>&1 || fail "Docker 没运行或没权限。执行：sudo systemctl start docker；
   如果是权限问题：sudo usermod -aG docker \$USER 然后注销重新登录"
command -v python3 >/dev/null || fail "没装 python3"
python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' || fail "需要 Python 3.10 及以上"
python3 -c 'import venv, ensurepip' 2>/dev/null || fail "缺少 venv。执行：sudo apt install python3-venv"
echo "✅ 环境检查通过"

step "2/6 启动 Qdrant"
docker compose up -d
for _ in $(seq 1 30); do
  curl -fsS http://localhost:6333/ >/dev/null 2>&1 && break
  sleep 1
done
curl -fsS http://localhost:6333/ >/dev/null || fail "Qdrant 没起来，用 docker logs qdrant-semantic-map 看原因"
echo "✅ Qdrant 已运行：http://localhost:6333/dashboard"

step "3/6 安装 Python 依赖（第一次较慢）"
[ -d .venv ] || python3 -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install -q --upgrade pip
python -c 'import torch' 2>/dev/null || pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -q -r requirements.txt
echo "✅ 依赖安装完成"

step "4/6 下载 BGE-M3 模型（约 2.3GB，只下载一次）"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
python -c 'import embedder; embedder.load_model()'
echo "✅ 模型就绪"

step "5/6 建库"
python build_index.py

step "6/6 测试查询"
# 一个进程里查多句，模型只加载一次
python - <<'EOF'
from query import search
for q in ["我想喝水", "机器人没电了", "一会儿要开会", "今天股票涨了吗"]:
    r = search(q)
    if r["found"]:
        b = r["best"]
        print(f"{q:　<8} → {b['title']}  x={b['x']} y={b['y']} yaw={b['yaw']}  score={b['score']}")
    else:
        print(f"{q:　<8} → 没找到（最高分 {r['candidates'][0]['score'] if r['candidates'] else 0}）")
EOF

echo
echo "🎉 部署完成。以后使用："
echo "   cd $(pwd) && source .venv/bin/activate"
echo "   python query.py \"你想去的地方\""
