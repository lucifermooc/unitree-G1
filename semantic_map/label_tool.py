"""语义地图标注工具：在浏览器里的地图上点选地点，自动生成 semantic_map.json 并重建语义库。

用法：
    python label_tool.py --map ../sim/maps/tb3_world.yaml
    然后浏览器打开 http://localhost:8000

    --map    建图时保存的地图 .yaml（和同名 .pgm 放在一起），G1 建图后换成它的地图
    --data   要编辑的地点文件，默认 config.DATA_FILE（data/semantic_map.json）
    --host   默认 127.0.0.1 只允许本机访问；想用平板/手机打开就设 0.0.0.0
    --port   默认 8000

"保存"只写 JSON 文件；"重建语义库"会用 BGE-M3 重新生成向量写进 Qdrant（需要 Qdrant 在运行）。
"""
import argparse
import base64
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import config
from build_index import validate_places

HTML_FILE = Path(__file__).resolve().parent / "label_tool.html"


def load_map(yaml_path):
    """读取 ROS 地图（map_server 格式的 .yaml + .pgm），不依赖 ROS 和第三方库。"""
    yaml_path = Path(yaml_path).resolve()
    meta = {}
    for line in yaml_path.read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            key, value = line.split(":", 1)
            meta[key.strip()] = value.strip()
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]
    image = Path(meta["image"])
    if not image.is_absolute():
        image = yaml_path.parent / image

    data = image.read_bytes()
    # PGM(P5) 文件头：P5 <宽> <高> <最大值>，中间可能夹着 # 注释
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b"P5" or int(tokens[3]) > 255:
        raise ValueError(f"只支持 8 位二进制 PGM（P5）：{image}")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[pos + 1:pos + 1 + width * height]
    if meta.get("negate", "0") == "1":
        pixels = bytes(255 - p for p in pixels)
    return {
        "name": yaml_path.name,
        "width": width,
        "height": height,
        "resolution": float(meta["resolution"]),
        "origin": origin,
        "pixels": base64.b64encode(pixels).decode(),
    }


def write_places(path, places):
    """一个地点一行，方便人看和 git diff。"""
    lines = [json.dumps(p, ensure_ascii=False) for p in places]
    text = "[\n" + ",\n".join("  " + line for line in lines) + "\n]\n" if lines else "[]\n"
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def rebuild_index(places):
    """把地点重新向量化写进 Qdrant。第一次调用会加载 BGE-M3（CPU 上十几秒）。"""
    from qdrant_client import QdrantClient

    import build_index
    import embedder

    vectors = embedder.encode([build_index.place_to_text(p) for p in places]) if places else []
    client = QdrantClient(host=config.QDRANT_HOST, port=config.QDRANT_PORT)
    return build_index.build_index(client, places, vectors)


class Handler(BaseHTTPRequestHandler):
    map_info = None
    data_file = None
    lock = threading.Lock()  # 保存 / 重建 / 搜索 串行执行，避免同时写文件或同时加载模型

    def log_message(self, fmt, *args):  # 只打印出错的请求
        if args and str(args[1])[0] in "45":
            super().log_message(fmt, *args)

    def _send(self, status, body, content_type="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _read_places(self):
        path = Path(self.data_file)
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else []

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, HTML_FILE.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/map":
            self._send(200, self.map_info)
        elif self.path == "/api/places":
            self._send(200, {"places": self._read_places(), "data_file": str(self.data_file)})
        elif self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            with self.lock:
                if self.path == "/api/places":
                    places = validate_places(body.get("places"))
                    write_places(self.data_file, places)
                    self._send(200, {"ok": True, "count": len(places)})
                elif self.path == "/api/rebuild":
                    places = validate_places(self._read_places())
                    count = rebuild_index(places)
                    self._send(200, {"ok": True, "count": count, "collection": config.COLLECTION_NAME})
                elif self.path == "/api/search":
                    from query import search
                    self._send(200, search(str(body.get("query", "")).strip()))
                else:
                    self._send(404, {"error": "not found"})
        except ValueError as e:
            self._send(400, {"error": str(e)})
        except Exception as e:  # 例如 Qdrant 没启动、还没建库
            self._send(500, {"error": f"{type(e).__name__}: {e}"})


def main():
    parser = argparse.ArgumentParser(description="语义地图标注工具")
    parser.add_argument("--map", required=True, help="地图 .yaml 文件（建图时保存的）")
    parser.add_argument("--data", default=str(config.DATA_FILE), help="地点 JSON 文件")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    Handler.map_info = load_map(args.map)
    Handler.data_file = Path(args.data).resolve()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    shown_host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print(f"地图：{args.map}（{Handler.map_info['width']}×{Handler.map_info['height']}，"
          f"{Handler.map_info['resolution']} m/像素）")
    print(f"地点文件：{Handler.data_file}")
    print(f"Qdrant collection：{config.COLLECTION_NAME}")
    print(f"浏览器打开 http://{shown_host}:{args.port}   （Ctrl+C 退出）")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
