"""建库：读 JSON 地点数据 → title+description 转向量 → 连同坐标一起写入 Qdrant。

用法：
    python build_index.py
改了 data/semantic_map.json 之后重新跑一次即可（会先删掉旧库再重建）。
"""
import json
import sys

from qdrant_client import QdrantClient, models

import config
import embedder


def load_places(path):
    """读取并检查地点数据，有问题直接报出是哪一条。"""
    with open(path, encoding="utf-8") as f:
        places = json.load(f)

    seen_ids = set()
    for i, place in enumerate(places):
        where = f"第 {i + 1} 条地点"
        if not isinstance(place.get("id"), int) or place["id"] < 0:
            raise ValueError(f"{where}：id 必须是非负整数（Qdrant 的要求）")
        if place["id"] in seen_ids:
            raise ValueError(f"{where}：id={place['id']} 重复了")
        seen_ids.add(place["id"])
        for key in ("title", "description"):
            if not isinstance(place.get(key), str) or not place[key].strip():
                raise ValueError(f"{where}：缺少 {key} 或为空")
        pose = place.get("pose")
        if not isinstance(pose, dict):
            raise ValueError(f"{where}：缺少 pose")
        for key in ("x", "y", "yaw"):
            if not isinstance(pose.get(key), (int, float)):
                raise ValueError(f"{where}：pose.{key} 必须是数字")
    return places


def place_to_text(place):
    """只用 title + description 生成要向量化的文本，坐标不参与。"""
    return f"{place['title']}。{place['description']}"


def build_index(client, places, vectors):
    """重建 collection 并写入所有地点。payload 里保留完整记录（含坐标）。"""
    if client.collection_exists(config.COLLECTION_NAME):
        client.delete_collection(config.COLLECTION_NAME)

    client.create_collection(
        collection_name=config.COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=config.VECTOR_SIZE,
            distance=models.Distance.COSINE,
        ),
    )

    client.upsert(
        collection_name=config.COLLECTION_NAME,
        points=[
            models.PointStruct(id=place["id"], vector=vector, payload=place)
            for place, vector in zip(places, vectors)
        ],
    )
    return client.count(config.COLLECTION_NAME).count


def main():
    try:
        places = load_places(config.DATA_FILE)
    except ValueError as e:
        sys.exit(f"数据文件有问题：{e}")
    print(f"读取到 {len(places)} 个地点：{config.DATA_FILE}")

    texts = [place_to_text(p) for p in places]
    print("示例文本（送进模型的内容）：", texts[0])
    vectors = embedder.encode(texts)

    client = QdrantClient(host=config.QDRANT_HOST, port=config.QDRANT_PORT)
    count = build_index(client, places, vectors)
    print(f"已写入 {count} 个地点到 Qdrant collection '{config.COLLECTION_NAME}'")


if __name__ == "__main__":
    main()
