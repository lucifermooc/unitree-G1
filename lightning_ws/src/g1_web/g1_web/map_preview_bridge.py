"""建图实时预览：把 lightning 建图发布的原始栅格 /map 转成前端要的 /map_base64。

接口文档 2.2.4 的 /map_base64 在海尔版本里由 map_manager_server 输出，G1 版本已经没有这段代码，
前端订阅不到。本节点补上这一环，不改动任何原有节点：
  订阅 /map（nav_msgs/OccupancyGrid，lightning 建图时发布，reliable + transient_local）
  发布 /map_base64（nav_msgs/OccupancyGrid），data 里放 JPEG 的 base64 字符串的字节，
  编码方式与 map_manager_server 的 /get_map_image 完全相同，前端用同一段代码解码。
建图时地图可能有 1 MB 以上，按 publish_period（默认 1 秒）限频。
"""
import base64

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def grid_to_gray(msg):
    """OccupancyGrid → 灰度图（与 map_saver trinary 模式一致：254 空闲 / 0 障碍 / 205 未知），行序翻转成图片方向。"""
    data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    img = np.full(data.shape, 205, np.uint8)
    img[(data >= 0) & (data <= 19)] = 254   # free_thresh 0.196
    img[data >= 65] = 0                      # occupied_thresh 0.65
    return np.flipud(img)


def gray_to_payload(img):
    """与 map_manager_server.__map_pgm_to_grid 相同：着色 → JPEG → base64 → 字节列表。"""
    color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    color[np.where((color == [205, 205, 205]).all(axis=2))] = [170, 108, 82]
    color[np.where((color == [254, 254, 254]).all(axis=2))] = [200, 145, 127]
    ok, buf = cv2.imencode(".jpg", color)
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return np.frombuffer(base64.b64encode(buf.tobytes()), dtype=np.uint8).tolist()


class MapPreviewBridge(Node):
    def __init__(self):
        super().__init__("map_preview_bridge")
        self.period = float(self.declare_parameter("publish_period", 1.0).value)
        in_topic = self.declare_parameter("input_topic", "/map").value
        self.latest = None
        self.last_sent_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, in_topic, self.on_map, qos)
        self.pub = self.create_publisher(OccupancyGrid, "/map_base64", 1)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(f"{in_topic} -> /map_base64 every {self.period:.1f}s")

    def on_map(self, msg):
        self.latest = msg

    def tick(self):
        msg = self.latest
        if msg is None or msg.info.width == 0:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec, len(msg.data))
        if stamp == self.last_sent_stamp:
            return
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = gray_to_payload(grid_to_gray(msg))
        self.pub.publish(out)
        self.last_sent_stamp = stamp


def main():
    rclpy.init()
    node = MapPreviewBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
