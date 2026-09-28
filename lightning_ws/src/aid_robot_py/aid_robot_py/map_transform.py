import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import base64


class MapTransform(Node):

    def __init__(self):
        super().__init__('map_transform')
        self.subscription = self.create_subscription(OccupancyGrid, '/map',
                                                     self.callback, 1)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_base64', 1)

    def callback(self, msg):
        # 获取地图尺寸
        width = msg.info.width
        height = msg.info.height

        # 如果数据不完整，直接返回
        if width == 0 or height == 0 or len(msg.data) != width * height:
            self.get_logger().warn("Invalid map data: width={}, height={}, data_len={}".format(
                width, height, len(msg.data)))
            return

        data = msg.data
        array2d = np.zeros((height, width), dtype=np.uint8)

        for i in range(height):
            for j in range(width):
                index = i * width + j
                h = height - i - 1
                if data[index] == -1:
                    array2d[h, j] = 100
                else:
                    array2d[h, j] = max(0, min(255, (100 - data[index]) * 2))

        # 转灰度再转 BGR
        img = cv2.cvtColor(array2d, cv2.COLOR_GRAY2BGR)

        img[np.where((img == [100, 100, 100]).all(axis=2))] = [170, 108, 82]
        img[np.where(((img > [100, 100, 100]) &
                    (img <= [150, 150, 150])).all(axis=2))] = [185, 125, 100]
        img[np.where((img > [150, 150, 150]).all(axis=2))] = [200, 145, 127]

        retval, buffer = cv2.imencode('.jpg', img)
        if not retval:
            self.get_logger().error("JPEG encoding failed")
            return

        base64_data = base64.b64encode(buffer.tobytes())
        int_array = np.frombuffer(base64_data, dtype=np.uint8)

        send_msg = OccupancyGrid()
        send_msg.header = msg.header
        send_msg.info = msg.info
        send_msg.data = int_array.tolist()

        self.publisher.publish(send_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
