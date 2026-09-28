"""查询：输入一句话 → 转向量 → 在 Qdrant 里找最相似的地点 → 输出坐标 JSON。

用法：
    python query.py "我想喝水"      # 查一次
    python query.py                 # 交互模式，可以连续输入，输入 q 退出

在别的程序（比如导航）里用：
    from query import search
    result = search("去充电")
    if result["found"]:
        x, y, yaw = result["best"]["x"], result["best"]["y"], result["best"]["yaw"]
"""
import json
import sys

from qdrant_client import QdrantClient

import config
import embedder

_client = None


def get_client():
    global _client
    if _client is None:
        _client = QdrantClient(host=config.QDRANT_HOST, port=config.QDRANT_PORT)
    return _client


def _hit_to_dict(point):
    place = point.payload
    return {
        "id": place["id"],
        "title": place["title"],
        "x": place["pose"]["x"],
        "y": place["pose"]["y"],
        "yaw": place["pose"]["yaw"],
        "frame_id": place.get("frame_id", "map"),
        "score": round(point.score, 4),
    }


def search(text, top_k=3, client=None):
    """返回 {"query", "found", "best", "candidates"}；分数不够阈值时 found=False、best=None。"""
    client = client or get_client()
    query_vector = embedder.encode([text])[0]
    hits = client.query_points(
        collection_name=config.COLLECTION_NAME,
        query=query_vector,
        limit=top_k,
        with_payload=True,
    ).points

    candidates = [_hit_to_dict(p) for p in hits]
    found = bool(candidates) and candidates[0]["score"] >= config.SCORE_THRESHOLD
    return {
        "query": text,
        "found": found,
        "best": candidates[0] if found else None,
        "candidates": candidates,
    }


def _print_result(text):
    print(json.dumps(search(text), ensure_ascii=False, indent=2))


def main():
    if len(sys.argv) > 1:
        _print_result(" ".join(sys.argv[1:]))
        return

    embedder.load_model()
    print("输入一句话查找地点，输入 q 退出")
    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if text.lower() in ("q", "quit", "exit"):
            break
        if text:
            _print_result(text)


if __name__ == "__main__":
    main()
