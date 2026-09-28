#!/usr/bin/env python3
"""Local-only fake Unitree API endpoint for bridge integration checks."""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response


class MockUnitreeApi(Node):
    def __init__(self):
        super().__init__('mock_unitree_api')
        self.publisher = self.create_publisher(Response, '/api/sport/response', 1)
        self.subscription = self.create_subscription(
            Request, '/api/sport/request', self.on_request, 1)

    def on_request(self, request):
        response = Response()
        response.header.identity = request.header.identity
        response.header.status.code = 0
        self.publisher.publish(response)
        print(
            f'api_id={request.header.identity.api_id} '
            f'parameter={request.parameter}',
            flush=True,
        )


def main():
    rclpy.init()
    node = MockUnitreeApi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
