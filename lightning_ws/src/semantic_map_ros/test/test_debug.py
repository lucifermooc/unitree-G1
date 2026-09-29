"""输入检查和调试记录的单元测试（不需要模型、ROS、Qdrant）。"""
import json

from semantic_map_ros.core import inspect_text, parse_request
from semantic_map_ros.debug_log import DebugLog, visible


def test_parse_request_plain_and_json():
    assert parse_request("我想喝水", "search") == ("我想喝水", "search")
    assert parse_request('{"text": "去厨房", "source": "asr"}', "go") == ("去厨房", "asr")
    assert parse_request('{"text": "去厨房"}', "topic") == ("去厨房", "topic")
    # 不是合法 JSON 或没有 text 字段：整句当普通文本
    assert parse_request("{随便说的", "search") == ("{随便说的", "search")
    assert parse_request('{"data": 1}', "search") == ('{"data": 1}', "search")


def test_inspect_text_reports_problems_without_changing_words():
    assert inspect_text("我想喝水") == ("我想喝水", [])
    text, issues = inspect_text(" 我想​喝水\n")
    assert text == "我想喝水"
    assert any("首尾有空白" in i for i in issues) and any("零宽空格" in i for i in issues)
    text, issues = inspect_text("我想\n喝水")
    assert text == "我想\n喝水" and any("中间有换行" in i for i in issues)
    assert any("乱码" in i for i in inspect_text("�水")[1])
    assert inspect_text("​ ")[0] == "" and "清理后是空的" in inspect_text("​ ")[1]


def test_visible_marks_hidden_characters():
    assert visible(" 我想​喝水\n") == "␠我想⟨U+200B⟩喝水\\n"
    assert visible("我想喝水") == "我想喝水"
    assert visible("  ") == "␠␠"


def test_debug_log_history_file_and_rotation(tmp_path):
    path = tmp_path / "sub" / "log.jsonl"
    log = DebugLog(str(path), history_size=3, max_bytes=60)  # 每行约 30 字节，写几行就会滚动
    for i in range(5):
        log.add({"i": i, "raw": "我想喝水"})
    assert [r["i"] for r in log.recent()] == [2, 3, 4]
    lines = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()]
    rotated = [json.loads(l) for l in (tmp_path / "sub" / "log.jsonl.1").read_text(encoding="utf-8").splitlines()]
    assert [r["i"] for r in rotated + lines] == list(range(5))[-len(rotated + lines):]
    assert lines[-1]["raw"] == "我想喝水"  # 中文原样写入，不转义


def test_debug_log_without_file():
    log = DebugLog("", history_size=2)
    line = log.add({"a": 1})
    assert json.loads(line) == {"a": 1} and log.recent() == [{"a": 1}]
