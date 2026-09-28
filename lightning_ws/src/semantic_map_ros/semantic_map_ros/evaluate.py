"""评测语义匹配效果、确定阈值：用地图数据库里的真实点位，测一批"相关的话"和"无关的话"。

不需要 ROS、不需要 Qdrant 服务（用内存版 Qdrant），只需要模型：
    python3 -m semantic_map_ros.evaluate --queries my_queries.json                 # 默认 ~/maps/db.sqlite 的当前地图
    python3 -m semantic_map_ros.evaluate --db ~/maps/db.sqlite --map-id 3 --queries my_queries.json

my_queries.json 格式（参考 config/eval_queries_example.json）：
    {"positive": [{"query": "我想喝水", "expect": "茶水间"}, ...],
     "negative": ["今天股票涨了吗", ...]}
输出每句话的第一名和分数、准确率，以及"相关句子最低分 / 无关句子最高分"和推荐阈值，
把推荐值填进 config/semantic_map.yaml 的 score_threshold。
"""
import argparse
import json
import os
import sqlite3

from .core import SemanticIndex, parse_point_rows


def load_places(db, map_id):
    with sqlite3.connect(os.path.expanduser(db)) as conn:
        conn.row_factory = sqlite3.Row
        if map_id is None:
            row = conn.execute("select id from current_map").fetchone()
            if row is None:
                raise SystemExit("数据库里没有当前地图，用 --map-id 指定")
            map_id = row["id"]
        rows = [dict(r) for r in conn.execute(
            "select id, frame_id, point_list from waypoint_node where map_id=? and creator_id=0", (map_id,))]
    return map_id, parse_point_rows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--db", default="~/maps/db.sqlite")
    ap.add_argument("--map-id", type=int)
    ap.add_argument("--queries", required=True)
    ap.add_argument("--model", default="BAAI/bge-m3")
    ap.add_argument("--fp16", action="store_true")
    ap.add_argument("--threshold", type=float, default=0.52, help="当前使用的阈值，只用于对比")
    args = ap.parse_args()

    from qdrant_client import QdrantClient

    from .embedder import Embedder

    map_id, places = load_places(args.db, args.map_id)
    if not places:
        raise SystemExit(f"地图 {map_id} 没有点位")
    cases = json.load(open(args.queries, encoding="utf-8"))
    index = SemanticIndex(QdrantClient(":memory:"), Embedder(args.model, args.fp16).encode, "eval")
    index.rebuild(map_id, places)
    print(f"地图 {map_id}：{len(places)} 个点位 {[p['name'] for p in places]}\n")

    print("【相关的话】期望 → 实际第一名（分数）")
    pos, wrong = [], []
    for c in cases.get("positive", []):
        top = index.search(map_id, c["query"], threshold=0.0)["candidates"][0]
        ok = top["name"] == c["expect"]
        pos.append(top["score"])
        wrong += [] if ok else [c["query"]]
        print(f"  {'✅' if ok else '❌'} {c['query']}  期望 {c['expect']} → {top['name']} {top['score']:.3f}")
    print("\n【无关的话】第一名（分数，越低越好）")
    neg = []
    for q in cases.get("negative", []):
        top = index.search(map_id, q, threshold=0.0)["candidates"][0]
        neg.append(top["score"])
        print(f"  {q} → {top['name']} {top['score']:.3f}")

    print("\n【统计】")
    if pos:
        print(f"  排序正确 {len(pos) - len(wrong)}/{len(pos)}" + (f"，错误：{wrong}" if wrong else ""))
        print(f"  相关的话最低分 {min(pos):.3f}")
    if neg:
        print(f"  无关的话最高分 {max(neg):.3f}")
    if pos and neg:
        if min(pos) > max(neg):
            print(f"  能完全分开，推荐阈值 ≈ {(min(pos) + max(neg)) / 2:.2f}")
        else:
            print("  有重叠：把分数低的相关说法写进点位描述，或接受少量误判后取中间值")
        print(f"  当前阈值 {args.threshold}：误拒相关 {sum(s < args.threshold for s in pos)} 句，"
              f"误放无关 {sum(s >= args.threshold for s in neg)} 句")


if __name__ == "__main__":
    main()
