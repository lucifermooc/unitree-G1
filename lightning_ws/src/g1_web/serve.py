#!/usr/bin/env python3
"""网页控制台的静态文件服务（不需要 ROS，任何装了 Python 3 的电脑都能跑）。

    python3 serve.py              # http://localhost:8080，目录默认是旁边的 www/
    python3 serve.py 8081 --directory /path/to/www

和 `python3 -m http.server` 一样，只多发 Cache-Control: no-cache：否则浏览器会按启发式缓存直接用旧的
app.js / style.css，更新代码后刷新页面也看不到变化。
"""
import argparse
import functools
import http.server
import os


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")  # 每次都向服务器确认（没变就 304，不浪费流量）
        super().end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", type=int, default=8080)
    ap.add_argument("--directory", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "www"))
    ap.add_argument("--bind", default="", help="监听地址，默认所有网卡")
    a = ap.parse_args()
    handler = functools.partial(NoCacheHandler, directory=a.directory)
    with http.server.ThreadingHTTPServer((a.bind, a.port), handler) as httpd:
        print(f"serving {a.directory} on http://{a.bind or '0.0.0.0'}:{a.port}", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
