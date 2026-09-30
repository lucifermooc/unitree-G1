"""semantic_map_ros.core 单元测试：不需要模型、不需要 ROS、不需要 Qdrant 服务。

用 QdrantClient(":memory:") 和按字符二元组哈希的假编码器代替 BGE-M3。
    python3 -m pytest src/semantic_map_ros/test -q
"""
import hashlib
import json
import math

import pytest

from semantic_map_ros.core import SemanticIndex, parse_point_rows, place_text, signature, yaw_of

qdrant_client = pytest.importorskip("qdrant_client")


def fake_encode(texts):
    out = []
    for t in texts:
        v = [0.0] * 1024
        for a, b in zip(t, t[1:]):
            v[int(hashlib.md5((a + b).encode()).hexdigest(), 16) % 1024] += 1
        n = math.sqrt(sum(x * x for x in v)) or 1
        out.append([x / n for x in v])
    return out


def row(pid, name, desc=None, x=0.0, y=0.0, qz=0.0, qw=1.0, as_string=True):
    data = {"position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}, "name": name}
    if desc is not None:
        data["description"] = desc
    return {"id": pid, "frame_id": "map", "point_list": json.dumps(data, ensure_ascii=False) if as_string else data}


def test_parse_real_backend_format():
    # map_manager_server 返回的 message 本身是 JSON 字符串，point_list 也是 JSON 字符串
    message = json.dumps([row(1, "茶水间", "可以接水喝水", 1.5, 2.0, 0.7071068, 0.7071068),
                          row(2, "前台", as_string=False)])
    places = parse_point_rows(message)
    assert [p["name"] for p in places] == ["茶水间", "前台"]
    assert places[0]["description"] == "可以接水喝水" and places[1]["description"] == ""
    assert places[0]["x"] == 1.5 and abs(yaw_of(places[0]["orientation"]) - math.pi / 2) < 1e-6


def test_parse_skips_bad_rows():
    bad = [{"id": 3, "frame_id": "map", "point_list": "not json"},
           {"id": 4, "frame_id": "map", "point_list": json.dumps({"name": "没坐标"})},
           row(5, "   "), row(6, "好的")]
    assert [p["id"] for p in parse_point_rows(json.dumps(bad))] == [6]
    assert parse_point_rows("[]") == []


def test_place_text_and_signature():
    a = parse_point_rows(json.dumps([row(1, "茶水间", "喝水")]))
    b = parse_point_rows(json.dumps([row(1, "茶水间", "喝咖啡")]))
    c = parse_point_rows(json.dumps([row(1, "茶水间", "喝水", x=1.0)]))
    assert place_text(a[0]) == "茶水间。喝水"
    assert place_text(parse_point_rows(json.dumps([row(2, "前台")]))[0]) == "前台"
    assert signature(a) != signature(b) != signature(c)
    assert signature(a) == signature(list(reversed(a)))


def test_index_rebuild_search_threshold():
    idx = SemanticIndex(qdrant_client.QdrantClient(":memory:"), fake_encode, "t")
    places = parse_point_rows(json.dumps([
        row(1, "茶水间", "有饮水机，可以接水、喝水、泡茶", 3.0, 1.0),
        row(2, "充电桩", "机器人没电了回这里充电", -1.0, 0.5, 1.0, 0.0),
        row(3, "会议室", "开会和汇报的地方", 2.0, 2.5),
    ]))
    assert not idx.is_fresh(7, places)
    assert idx.rebuild(7, places) == 3
    assert idx.is_fresh(7, places)

    r = idx.search(7, "我想喝水", threshold=0.1)
    assert r["found"] and r["best"]["name"] == "茶水间" and r["best"]["x"] == 3.0
    assert len(r["candidates"]) == 3
    r = idx.search(7, "机器人没电了", threshold=0.1)
    assert r["best"]["name"] == "充电桩" and abs(abs(r["best"]["yaw"]) - math.pi) < 1e-3
    r = idx.search(7, "今天股票涨了吗", threshold=0.1)
    assert not r["found"] and r["best"] is None

    # 点位变了 → 不新鲜 → 重建后能搜到新点，删掉的点搜不到
    places2 = places[:2] + parse_point_rows(json.dumps([row(4, "打印室", "打印复印文件")]))
    assert not idx.is_fresh(7, places2)
    idx.rebuild(7, places2)
    names = [c["name"] for c in idx.search(7, "打印文件", threshold=0.1)["candidates"]]
    assert names[0] == "打印室" and "会议室" not in names


def test_each_map_has_its_own_collection():
    idx = SemanticIndex(qdrant_client.QdrantClient(":memory:"), fake_encode, "t")
    idx.rebuild(1, parse_point_rows(json.dumps([row(1, "茶水间", "喝水")])))
    idx.rebuild(2, parse_point_rows(json.dumps([row(1, "仓库", "放工具")])))
    assert idx.search(1, "喝水", 0.0)["best"]["name"] == "茶水间"
    assert idx.search(2, "喝水", 0.0)["best"]["name"] == "仓库"   # 地图 2 只有仓库
    idx.rebuild(3, [])
    assert idx.client.count(idx.collection(3)).count == 0


def test_local_path_mode_persists(tmp_path):
    # 没有 Qdrant 服务的机器（8550）用 QdrantClient(path=...)：关掉重开后向量还在（新进程第一次仍会按签名重建一次）
    places = parse_point_rows(json.dumps([row(1, "茶水间", "喝水"), row(2, "仓库", "放工具")]))
    c1 = qdrant_client.QdrantClient(path=str(tmp_path))
    SemanticIndex(c1, fake_encode, "t").rebuild(5, places)
    c1.close()
    c2 = qdrant_client.QdrantClient(path=str(tmp_path))
    idx = SemanticIndex(c2, fake_encode, "t")
    assert c2.count(idx.collection(5)).count == 2
    assert idx.search(5, "喝水", 0.0)["best"]["name"] == "茶水间"
    c2.close()
