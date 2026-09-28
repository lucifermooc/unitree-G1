"""语义地图的全部配置，换机器（笔记本 → Thor）时只改这里或用环境变量覆盖。"""
import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# 地点数据文件（坐标 + title + description）
DATA_FILE = Path(os.getenv("SEMANTIC_MAP_DATA", BASE_DIR / "data" / "semantic_map.json"))

# Qdrant 服务地址（docker compose 启动后默认就是 localhost:6333）
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", "6333"))
COLLECTION_NAME = os.getenv("SEMANTIC_MAP_COLLECTION", "semantic_map")

# BGE-M3 模型
MODEL_NAME = os.getenv("EMBED_MODEL", "BAAI/bge-m3")
VECTOR_SIZE = 1024  # BGE-M3 dense 向量固定 1024 维
# 笔记本只有 CPU → False；部署到 Thor（有 CUDA）时设 USE_FP16=1
USE_FP16 = os.getenv("USE_FP16", "0") == "1"

# 相似度低于这个分数就认为"没找到"，防止随便一句话机器人也乱跑。
# 0.45 是初始值，用自己的数据实测后再调。
SCORE_THRESHOLD = float(os.getenv("SCORE_THRESHOLD", "0.45"))
