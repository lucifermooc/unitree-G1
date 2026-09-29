"""语义地图调试记录：每次请求一条，内存里留最近 N 条，同时追加写进 JSONL 文件（不依赖 ROS）。

文件每行一条 JSON，超过 max_bytes 时把旧文件改名为 <文件名>.1 再重新写（只留一份旧的）。
"""
import collections
import json
import os
import threading
import unicodedata


def visible(raw):
    """把看不见的字符写出来，方便肉眼对比：换行 → \\n，零宽空格 → ⟨U+200B⟩，首尾空格 → ␠。"""
    out = []
    for c in raw:
        if c == "\n":
            out.append("\\n")
        elif c == "\r":
            out.append("\\r")
        elif c == "\t":
            out.append("\\t")
        elif unicodedata.category(c) in ("Cc", "Cf") or c in "　�":
            out.append(f"⟨U+{ord(c):04X}⟩")
        else:
            out.append(c)
    s = "".join(out)
    lead = len(s) - len(s.lstrip(" "))
    trail = len(s) - len(s.rstrip(" "))
    return "␠" * lead + s.strip(" ") + "␠" * trail if s.strip(" ") else "␠" * len(s)


class DebugLog:
    def __init__(self, path="", history_size=200, max_bytes=20 * 1024 * 1024):
        self.path = os.path.expanduser(path) if path else ""
        self.max_bytes = max_bytes
        self.history = collections.deque(maxlen=history_size)
        self._lock = threading.Lock()
        if self.path:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)

    def add(self, record):
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            self.history.append(record)
            if not self.path:
                return line
            try:
                if os.path.exists(self.path) and os.path.getsize(self.path) > self.max_bytes:
                    os.replace(self.path, self.path + ".1")
                with open(self.path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass  # 写日志失败不能影响搜索和导航
        return line

    def recent(self, n=None):
        with self._lock:
            items = list(self.history)
        return items if n is None else items[-n:]
