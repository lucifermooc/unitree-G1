"""语义地图核心逻辑（不依赖 ROS，方便单元测试）。

数据来源是地图数据库里的位置点（waypoint_node 表），由 map_manager_server 的
/get_map_point_list 返回。每个点的 point_list 是一个 JSON 字符串：
    {"position": {...}, "orientation": {...}, "name": "茶水间", "description": "可以接水、喝水"}
description 是语义地图新增的可选字段，后端原样存储；没有描述时只用名称做语义匹配。
坐标不参与向量化，只放在 payload 里，搜到后原样取出来导航。
"""
import hashlib
import json
import math
import unicodedata

VECTOR_SIZE = 1024  # BGE-M3 dense 向量维度


def parse_point_rows(message):
    """/get_map_point_list 的 message（JSON 字符串）→ 地点列表。格式不对的点跳过。"""
    rows = json.loads(message) if isinstance(message, str) else message
    places = []
    for row in rows or []:
        try:
            data = row["point_list"]
            if isinstance(data, str):
                data = json.loads(data)
            pos, ori = data["position"], data.get("orientation") or {"x": 0, "y": 0, "z": 0, "w": 1}
            name = str(data.get("name", "")).strip()
            if not name:
                continue
            places.append({
                "id": int(row["id"]),
                "name": name,
                "description": str(data.get("description", "") or "").strip(),
                "frame_id": row.get("frame_id") or "map",
                "x": float(pos["x"]),
                "y": float(pos["y"]),
                "z": float(pos.get("z", 0.0)),
                "orientation": {k: float(ori.get(k, 0.0)) for k in ("x", "y", "z", "w")},
            })
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    return places


def parse_request(data, default_source):
    """请求内容 → (原始文本, 来源)。

    data 可以是一句普通文本，也可以是 JSON：{"text": "我想喝水", "source": "asr"}，
    source 用来在调试记录里区分是谁发来的；不带 source 时用 default_source。
    """
    if isinstance(data, str) and data.lstrip().startswith("{"):
        try:
            obj = json.loads(data)
        except json.JSONDecodeError:
            obj = None
        if isinstance(obj, dict) and isinstance(obj.get("text"), str):
            source = str(obj.get("source") or default_source).strip() or default_source
            return obj["text"], source
    return data, default_source


def _char_name(ch):
    return {" ": "空格", "\t": "制表符", "\n": "换行", "\r": "回车", "\u3000": "全角空格",
            "\ufeff": "BOM", "\u200b": "零宽空格", "\ufffd": "乱码替换符"}.get(ch, f"U+{ord(ch):04X}")


def inspect_text(raw):
    """检查收到的原始文本，返回 (送进模型的文本, 问题列表)。

    清理只做两件事：去掉不可见的控制/格式字符，去掉首尾空白。不改错字、不删标点。
    问题列表用来判断"是发来的数据有问题，还是匹配有问题"。
    """
    issues = []
    edges = raw[:len(raw) - len(raw.lstrip())] + raw[len(raw.rstrip()):]
    if edges:
        issues.append("首尾有空白：" + "、".join(sorted({_char_name(c) for c in edges})))
    hidden = {c for c in raw if unicodedata.category(c) in ("Cc", "Cf") and c not in "\t\n\r"}
    inner_breaks = {c for c in raw.strip() if c in "\t\n\r"}
    if hidden:
        issues.append("含不可见字符：" + "、".join(sorted(_char_name(c) for c in hidden)))
    if inner_breaks:
        issues.append("中间有换行/制表符：" + "、".join(sorted(_char_name(c) for c in inner_breaks)))
    if "\ufffd" in raw:
        issues.append("含乱码替换符（编码错误）")
    text = "".join(c for c in raw if unicodedata.category(c) not in ("Cc", "Cf") or c in "\t\n\r").strip()
    if not text:
        issues.append("清理后是空的")
    return text, issues


def place_text(place):
    """送进模型的文本：名称 + 描述（坐标不参与）。"""
    if place["description"]:
        return f"{place['name']}。{place['description']}"
    return place["name"]


def yaw_of(orientation):
    o = orientation
    return math.atan2(2 * (o["w"] * o["z"] + o["x"] * o["y"]), 1 - 2 * (o["y"] ** 2 + o["z"] ** 2))


def signature(places):
    """点位内容的指纹：点位有增删改时变化，用来判断语义库要不要重建。"""
    key = [(p["id"], place_text(p), p["x"], p["y"], p["orientation"]["z"], p["orientation"]["w"])
           for p in sorted(places, key=lambda p: p["id"])]
    return hashlib.sha1(json.dumps(key, ensure_ascii=False).encode()).hexdigest()


class SemanticIndex:
    """一张地图对应 Qdrant 里的一个 collection。

    client: qdrant_client.QdrantClient（测试时可以用 QdrantClient(":memory:")）
    encode: 函数，list[str] → list[list[float]]
    """

    def __init__(self, client, encode, collection_prefix="semantic_map"):
        self.client = client
        self.encode = encode
        self.prefix = collection_prefix
        self._signatures = {}  # map_id → 已写入的点位指纹

    def collection(self, map_id):
        return f"{self.prefix}_{map_id}"

    def is_fresh(self, map_id, places):
        return self._signatures.get(map_id) == signature(places) and \
            self.client.collection_exists(self.collection(map_id))

    def rebuild(self, map_id, places):
        """删掉旧的 collection 再整体重建，返回写入的点数。"""
        from qdrant_client import models

        name = self.collection(map_id)
        if self.client.collection_exists(name):
            self.client.delete_collection(name)
        self.client.create_collection(
            collection_name=name,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        if places:
            vectors = self.encode([place_text(p) for p in places])
            self.client.upsert(collection_name=name, points=[
                models.PointStruct(id=p["id"], vector=v, payload=p) for p, v in zip(places, vectors)
            ])
        self._signatures[map_id] = signature(places)
        return len(places)

    def search(self, map_id, text, threshold, top_k=3):
        """返回 {"query", "found", "best", "candidates"}；最高分低于阈值时 found=False。"""
        hits = self.client.query_points(
            collection_name=self.collection(map_id),
            query=self.encode([text])[0],
            limit=top_k,
            with_payload=True,
        ).points
        candidates = []
        for h in hits:
            p = h.payload
            candidates.append({
                "id": p["id"], "name": p["name"], "description": p["description"],
                "frame_id": p["frame_id"], "x": p["x"], "y": p["y"], "z": p.get("z", 0.0),
                "yaw": round(yaw_of(p["orientation"]), 4), "orientation": p["orientation"],
                "score": round(float(h.score), 4),
            })
        found = bool(candidates) and candidates[0]["score"] >= threshold
        return {"query": text, "found": found, "best": candidates[0] if found else None,
                "candidates": candidates}
