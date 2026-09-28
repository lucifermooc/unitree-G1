"""评测 + 调阈值：用 data/eval_queries.json 里的测试句子检查准确率，并给出推荐阈值。

用法（先 build_index.py 建好库）：
    python evaluate.py
换成自己的地点后，把 eval_queries.json 里的句子也改成自己的再跑。
"""
import json

import config
from query import search


def main():
    with open(config.BASE_DIR / "data" / "eval_queries.json", encoding="utf-8") as f:
        cases = json.load(f)

    print("【相关句子】应该找到的地点 / 实际第一名 / 分数")
    pos_scores, wrong = [], []
    for case in cases["positive"]:
        top = search(case["query"])["candidates"][0]
        ok = top["title"] == case["expect"]
        pos_scores.append(top["score"])
        if not ok:
            wrong.append(case["query"])
        print(f"  {'✅' if ok else '❌'} {case['query']}  期望 {case['expect']}  → {top['title']}  {top['score']:.3f}")

    print("\n【不相关句子】第一名 / 分数（越低越好）")
    neg_scores = []
    for query in cases["negative"]:
        top = search(query)["candidates"][0]
        neg_scores.append(top["score"])
        print(f"  {query}  → {top['title']}  {top['score']:.3f}")

    lowest_pos, highest_neg = min(pos_scores), max(neg_scores)
    print("\n【统计】")
    print(f"  排序正确：{len(pos_scores) - len(wrong)}/{len(pos_scores)}" + (f"，错误：{wrong}" if wrong else ""))
    print(f"  相关句子最低分：{lowest_pos:.3f}")
    print(f"  不相关句子最高分：{highest_neg:.3f}")
    if lowest_pos > highest_neg:
        print(f"  两者能分开，推荐阈值 ≈ {(lowest_pos + highest_neg) / 2:.2f}（取中间）")
    else:
        print("  两者有重叠，阈值无法完全分开；看上面哪些句子重叠，改进 description 或接受少量误判")
    print(f"  当前 config.SCORE_THRESHOLD = {config.SCORE_THRESHOLD}")


if __name__ == "__main__":
    main()
