"""最小的 WebSocket 客户端（RFC 6455，只用标准库）。

为什么自己写：Thor 的系统 Python（ROS Jazzy 用的 /usr/bin/python3.12）没有 ``websockets``，
往 ~/.local 里 pip 安装会污染整套栈（见 lightning_ws/CLAUDE.md 的 PYTHONNOUSERSITE 坑），
apt 安装要 sudo，而 Ubuntu 24.04 的 python3-websockets 10.4 也没有同步接口。
lightnav-serve 的协议是"一问一答的 JSON 文本帧"，用到的 WebSocket 功能很少：握手、发掩码文本帧、
收文本帧（含分片）、回 ping、处理 close。这里就只实现这些。

本模块也给 :mod:`g1_vln.fake_server`（假推理服务端）提供服务端用的握手和收发帧函数。
"""

from __future__ import annotations

import base64
import hashlib
import os
import socket
import ssl
import struct
import threading
import time
from typing import Optional, Tuple, Union
from urllib.parse import urlparse

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA
DEFAULT_MAX_MESSAGE = 64 * 1024 * 1024   # 与 lightnav-serve 的 max_size 一致


class WsError(Exception):
    """WebSocket 层的错误（协议违例、握手失败等）。"""


class WsHandshakeError(WsError):
    pass


class WsClosed(WsError):
    def __init__(self, code: int = 1006, reason: str = ""):
        super().__init__(f"连接已关闭（code={code}{'，' + reason if reason else ''}）")
        self.code = code
        self.reason = reason


class WsTimeout(WsError, TimeoutError):
    pass


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + GUID).encode("ascii")).digest()).decode("ascii")


def parse_ws_url(url: str) -> Tuple[str, str, int, str]:
    """返回 (scheme, host, port, resource)。只接受 ws:// 和 wss://。"""
    u = urlparse(str(url).strip())
    if u.scheme not in ("ws", "wss"):
        raise ValueError(f"推理服务地址必须以 ws:// 或 wss:// 开头：{url!r}")
    if not u.hostname:
        raise ValueError(f"推理服务地址缺少主机名：{url!r}")
    try:
        port = u.port
    except ValueError as e:
        raise ValueError(f"推理服务地址端口不对：{url!r}") from e
    if port is None:
        port = 443 if u.scheme == "wss" else 80
    resource = u.path or "/"
    if u.query:
        resource += "?" + u.query
    return u.scheme, u.hostname, port, resource


def mask_payload(data: bytes, mask: bytes) -> bytes:
    n = len(data)
    if n == 0:
        return b""
    m = (mask * (n // 4 + 1))[:n]
    return (int.from_bytes(data, "big") ^ int.from_bytes(m, "big")).to_bytes(n, "big")


def encode_frame(opcode: int, payload: bytes, masked: bool, fin: bool = True) -> bytes:
    """客户端发的帧必须加掩码（masked=True），服务端发的帧不加。"""
    head = bytearray([(0x80 if fin else 0) | opcode])
    n = len(payload)
    mbit = 0x80 if masked else 0
    if n < 126:
        head.append(mbit | n)
    elif n < 65536:
        head.append(mbit | 126)
        head += struct.pack("!H", n)
    else:
        head.append(mbit | 127)
        head += struct.pack("!Q", n)
    if masked:
        mask = os.urandom(4)
        head += mask
        payload = mask_payload(payload, mask)
    return bytes(head) + payload


class FrameIO:
    """在一个已握手的 socket 上收发帧。收和发各自加锁；abort() 可以从别的线程调用。"""

    def __init__(self, sock: socket.socket, client_side: bool, leftover: bytes = b"",
                 max_message: int = DEFAULT_MAX_MESSAGE):
        self.sock = sock
        self.client_side = client_side
        self.max_message = max_message
        self._rbuf = bytearray(leftover)
        self._send_lock = threading.Lock()
        self._recv_lock = threading.Lock()
        self.closed = False

    # ------------------------------------------------------------- send ----
    def send(self, opcode: int, payload: bytes, timeout: Optional[float] = None) -> None:
        if self.closed:
            raise WsClosed(1006, "连接已关闭")
        frame = encode_frame(opcode, payload, masked=self.client_side)
        with self._send_lock:
            try:
                self.sock.settimeout(timeout)
                self.sock.sendall(frame)
            except socket.timeout as e:
                self.closed = True
                raise WsTimeout("发送超时") from e
            except OSError as e:
                self.closed = True
                raise WsClosed(1006, f"发送失败：{e}") from e

    def send_text(self, text: str, timeout: Optional[float] = None) -> None:
        self.send(OP_TEXT, text.encode("utf-8"), timeout)

    # ------------------------------------------------------------- recv ----
    def _recv_exact(self, n: int, deadline: Optional[float]) -> bytes:
        while len(self._rbuf) < n:
            if deadline is None:
                self.sock.settimeout(None)
            else:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise WsTimeout("等待回包超时")
                self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(max(65536, n - len(self._rbuf)))
            except socket.timeout as e:
                raise WsTimeout("等待回包超时") from e
            except OSError as e:
                raise WsClosed(1006, f"接收失败：{e}") from e
            if not chunk:
                raise WsClosed(1006, "对方断开了连接")
            self._rbuf += chunk
        out = bytes(self._rbuf[:n])
        del self._rbuf[:n]
        return out

    def recv_frame(self, deadline: Optional[float]) -> Tuple[bool, int, bytes]:
        b1, b2 = self._recv_exact(2, deadline)
        fin = bool(b1 & 0x80)
        if b1 & 0x70:
            raise WsError("收到带扩展位的帧（不支持任何扩展）")
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        n = b2 & 0x7F
        if n == 126:
            n = struct.unpack("!H", self._recv_exact(2, deadline))[0]
        elif n == 127:
            n = struct.unpack("!Q", self._recv_exact(8, deadline))[0]
        if opcode >= 0x8 and (n > 125 or not fin):
            raise WsError("控制帧不合法")
        if n > self.max_message:
            raise WsError(f"消息太大（{n} 字节）")
        mask = self._recv_exact(4, deadline) if masked else None
        payload = self._recv_exact(n, deadline) if n else b""
        if mask:
            payload = mask_payload(payload, mask)
        return fin, opcode, payload

    def recv_message(self, timeout: Optional[float] = None) -> Union[str, bytes]:
        """收一条完整消息（自动拼分片、回 ping、忽略 pong）。文本返回 str，二进制返回 bytes。

        超时或出错后这个连接就不能再用了（可能停在半帧上），调用方应关闭它。
        """
        if self.closed:
            raise WsClosed(1006, "连接已关闭")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._recv_lock:
            try:
                return self._recv_message_locked(deadline)
            except WsError:
                self.closed = True
                raise

    def _recv_message_locked(self, deadline: Optional[float]) -> Union[str, bytes]:
        parts = []
        msg_op = None
        size = 0
        while True:
            fin, op, payload = self.recv_frame(deadline)
            if op == OP_CLOSE:
                code = struct.unpack("!H", payload[:2])[0] if len(payload) >= 2 else 1005
                reason = payload[2:].decode("utf-8", "replace")
                try:
                    self.send(OP_CLOSE, payload[:2], timeout=1.0)
                except WsError:
                    pass
                raise WsClosed(code, reason)
            if op == OP_PING:
                self.send(OP_PONG, payload, timeout=2.0)
                continue
            if op == OP_PONG:
                continue
            if op in (OP_TEXT, OP_BINARY):
                if msg_op is not None:
                    raise WsError("上一条消息还没收完就来了新消息")
                msg_op = op
            elif op == OP_CONT:
                if msg_op is None:
                    raise WsError("收到孤立的续帧")
            else:
                raise WsError(f"未知的帧类型 {op}")
            size += len(payload)
            if size > self.max_message:
                raise WsError(f"消息太大（>{self.max_message} 字节）")
            parts.append(payload)
            if fin:
                data = b"".join(parts)
                if msg_op == OP_TEXT:
                    try:
                        return data.decode("utf-8")
                    except UnicodeDecodeError as e:
                        raise WsError("文本帧不是 UTF-8") from e
                return data

    # ------------------------------------------------------------ close ----
    def close(self, code: int = 1000, timeout: float = 1.0) -> None:
        if not self.closed:
            try:
                self.send(OP_CLOSE, struct.pack("!H", code), timeout=timeout)
            except WsError:
                pass
        self.abort()

    def abort(self) -> None:
        """立刻断开（可从别的线程调用，用来打断阻塞中的 recv）。"""
        self.closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


def client_connect(url: str, timeout: float = 5.0, max_message: int = DEFAULT_MAX_MESSAGE) -> FrameIO:
    """连接并完成握手，返回客户端 FrameIO。失败抛 OSError / WsHandshakeError / ValueError。"""
    scheme, host, port, resource = parse_ws_url(url)
    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if scheme == "wss":
            sock = ssl.create_default_context().wrap_socket(sock, server_hostname=host)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host_hdr = host if ":" not in host else f"[{host}]"
        if not ((scheme == "ws" and port == 80) or (scheme == "wss" and port == 443)):
            host_hdr += f":{port}"
        req = (f"GET {resource} HTTP/1.1\r\n"
               f"Host: {host_hdr}\r\n"
               "Upgrade: websocket\r\n"
               "Connection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\n"
               "Sec-WebSocket-Version: 13\r\n"
               "User-Agent: g1_vln\r\n\r\n")
        sock.settimeout(timeout)
        sock.sendall(req.encode("ascii"))
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                raise WsHandshakeError("握手时对方关闭了连接（端口上跑的不是 WebSocket 服务？）")
            buf += chunk
            if len(buf) > 65536:
                raise WsHandshakeError("握手响应太长")
        head, _, rest = buf.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        status = lines[0].split(" ", 2)
        if len(status) < 2 or status[1] != "101":
            raise WsHandshakeError(f"握手失败：{lines[0]!r}")
        headers = {}
        for line in lines[1:]:
            k, _, v = line.partition(":")
            headers[k.strip().lower()] = v.strip()
        if headers.get("upgrade", "").lower() != "websocket":
            raise WsHandshakeError("握手响应缺少 Upgrade: websocket")
        if headers.get("sec-websocket-accept") != accept_key(key):
            raise WsHandshakeError("握手响应的 Sec-WebSocket-Accept 不对")
        if headers.get("sec-websocket-extensions"):
            raise WsHandshakeError("服务端要求了扩展（本客户端不支持）")
        return FrameIO(sock, client_side=True, leftover=rest, max_message=max_message)
    except BaseException:
        try:
            sock.close()
        except OSError:
            pass
        raise


def server_handshake(sock: socket.socket, timeout: float = 5.0) -> FrameIO:
    """服务端握手（给假服务端和测试用）。"""
    sock.settimeout(timeout)
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise WsHandshakeError("客户端在握手时断开")
        buf += chunk
        if len(buf) > 65536:
            raise WsHandshakeError("握手请求太长")
    head, _, rest = buf.partition(b"\r\n\r\n")
    lines = head.decode("latin-1").split("\r\n")
    headers = {}
    for line in lines[1:]:
        k, _, v = line.partition(":")
        headers[k.strip().lower()] = v.strip()
    key = headers.get("sec-websocket-key")
    if not lines[0].startswith("GET ") or not key or headers.get("upgrade", "").lower() != "websocket":
        sock.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
        raise WsHandshakeError("不是 WebSocket 握手请求")
    sock.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n").encode("ascii"))
    return FrameIO(sock, client_side=False, leftover=rest)
