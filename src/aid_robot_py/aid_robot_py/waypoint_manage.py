import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from aid_robot_msgs.srv import PatrolControl
from aid_robot_msgs.msg import AidTaskStatus
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from threading import Lock
import math


class WaypointFollower(Node):

    TASK_IDLE = 0
    TASK_WORKING = 1
    TASK_SUCCESS = 2
    TASK_FAILED = 3
    TASK_SUSPEND = 4
    TASK_CANCEL = 5

    TASK_POINT = 0
    TASK_CIRCLE = 1

    def __init__(self):
        super().__init__('waypoint_mange')

        # 状态
        self.task_status = AidTaskStatus()
        self.task_status.status = self.TASK_IDLE
        self.task_status.task_type = self.TASK_POINT
        self.status_lock = Lock()

        # 控制标志
        self.is_pause = False
        self.new_goal = False

        # 内部任务队列
        self.current_goal_list = []
        self.current_index = 0
        self._goal_handle = None  # 保存当前 goal handle

        # 订阅 & 发布
        self.nav_pose_sub = self.create_subscription(
            PoseStamped,
            '/nav_to_pose',
            self.nav_pose_callback,
            10
        )
        self.navi_path_sub = self.create_subscription(
            Path,
            '/patrol_path',
            self.navi_path_callback,
            1
        )
        self.task_status_pub = self.create_publisher(
            AidTaskStatus,
            '/task_status',
            5
        )

        # 服务
        self.patrol_control_srv = self.create_service(
            PatrolControl,
            '/patrol_control',
            self.patrol_control_callback
        )

        # Action Client
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # 定时器发布任务状态
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('WaypointFollower node started.')

    # ------------------ 状态发布 ------------------
    def publish_status(self):
        with self.status_lock:
            self.task_status_pub.publish(self.task_status)
    def quaternion_from_euler(self,ai, aj, ak):
        """
        欧拉角转四元数（绕 x=roll(ai), y=pitch(aj), z=yaw(ak)）
        输入：ai, aj, ak 单位弧度
        返回：[x, y, z, w]
        """
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0

        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)

        # 标准公式
        qw = ci * cj * ck + si * sj * sk
        qx = si * cj * ck - ci * sj * sk
        qy = ci * sj * ck + si * cj * sk
        qz = ci * cj * sk - si * sj * ck

        return [qx, qy, qz, qw]
    # ------------------ 方向计算 ------------------
    def calculate_orientation(self, from_pose, to_pose):
        dx = to_pose.position.x - from_pose.position.x
        dy = to_pose.position.y - from_pose.position.y
        yaw = math.atan2(dy, dx)
        q = self.quaternion_from_euler(0, 0, yaw)
        pose_orientation = PoseStamped().pose.orientation
        pose_orientation.x = q[0]
        pose_orientation.y = q[1]
        pose_orientation.z = q[2]
        pose_orientation.w = q[3]
        return pose_orientation

    def _is_valid_quaternion(self,q, eps=1e-6):
        """检查四元数是否合法（是否归一化，是否包含 NaN，是否全 0）"""
        # 检查是否包含 NaN
        if any(math.isnan(v) for v in [q.x, q.y, q.z, q.w]):
            return False

        # 检查是否全 0（一个全 0 四元数不是合法旋转）
        if abs(q.x) < eps and abs(q.y) < eps and abs(q.z) < eps and abs(q.w) < eps:
            return False

        # 检查是否单位长度
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        return abs(norm - 1.0) < eps
    # ------------------ 订阅回调 ------------------
    def nav_pose_callback(self, msg: PoseStamped):
        with self.status_lock:
            if self.task_status.status in [self.TASK_WORKING, self.TASK_SUSPEND]:
                self.get_logger().info('Nav task already running')
                return
            # if self._is_valid_quaternion(msg.pose.orientation) == False:
            #     self.get_logger().info('Nav task pose quaternion invaild')
            #     return
            self.task_status.task_type = self.TASK_POINT
            self.task_status.status = self.TASK_WORKING
        # return
        self.get_logger().info('Nav task pose received')
        self.current_goal_list = [msg]
        self.current_index = 0
        self.new_goal = True
        self.is_pause = False
        self.send_next_goal()

    def navi_path_callback(self, msg: Path):
        with self.status_lock:
            if self.task_status.status in [self.TASK_WORKING, self.TASK_SUSPEND]:
                self.get_logger().info('Nav task already running')
                return
            if len(msg.poses) == 0:
                self.get_logger().info(f'Nav task path received empty')
                return
            # for idx, pose_info in enumerate(msg.poses):
            #     if not self._is_valid_quaternion(pose_info.pose.orientation):
            #         self.get_logger().info(f'Nav task path Invalid quaternion at index {idx}')
            #         return
            self.task_status.task_type = self.TASK_CIRCLE
            self.task_status.status = self.TASK_WORKING

        self.get_logger().info(f'Nav task path received, size: {len(msg.poses)}')
        # return
        # 计算每段方向
        for i in range(len(msg.poses)):
            next_index = (i + 1) % len(msg.poses)
            msg.poses[i].pose.orientation = self.calculate_orientation(
                msg.poses[i].pose, msg.poses[next_index].pose
            )
        self.current_goal_list = msg.poses
        self.current_index = 0
        self.new_goal = True
        self.is_pause = False
        self.send_next_goal()

    # ------------------ 控制服务 ------------------
    def patrol_control_callback(self, request, response):
        with self.status_lock:
            if self.task_status.status not in [self.TASK_WORKING, self.TASK_SUSPEND]:
                response.success = True
                return response

        cmd = request.cmd.lower()
        self.get_logger().info(f'PatrolControlCallback: {cmd}')
        if cmd == 'cancel':
            self.new_goal = False
            self.cancel_all_goals()
            if self.is_pause == True:
                with self.status_lock:
                    self.task_status.status = self.TASK_CANCEL
            self.is_pause = False
        elif cmd == 'pause':
            with self.status_lock:
                self.task_status.status = self.TASK_SUSPEND
            self.is_pause = True
            self.new_goal = False
            self.cancel_all_goals()
        elif cmd == 'resume':
            with self.status_lock:
                self.task_status.status = self.TASK_WORKING
            self.is_pause = False
            self.new_goal = True
            self.send_next_goal()

        response.success = True
        return response

    # ------------------ Action Client ------------------
    def send_next_goal(self):
        # 等待当前 goal 完成
        if self.is_pause or not self.new_goal or self.current_index >= len(self.current_goal_list):
            return

        if self._goal_handle is not None:
            self.get_logger().warn(f'Goal {self.current_index} still in progress, skipping send_next_goal')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal_list[self.current_index]
        self.new_goal = False

        self.nav_to_pose_client.wait_for_server()
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()  # 保存 goal handle
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            with self.status_lock:
                self.task_status.status = self.TASK_FAILED
            self._goal_handle = None
            return
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass  # 可处理 feedback

    def __CirclePathPoint(self):
        if self.current_index >= len(self.current_goal_list) - 1:
            self.current_index = 0
            self.new_goal = True
            self.send_next_goal()
        else:
            self.current_index += 1
            self.new_goal = True
            self.send_next_goal()

    def result_callback(self, future):
        result = future.result()
        goal_status = result.status
        self.get_logger().info(f'Goal {self.current_index} status: {goal_status}')

        # 当前 goal 已完成，清空 handle
        self._goal_handle = None
        if self.is_pause == True:
            return

        with self.status_lock:
            if goal_status == GoalStatus.STATUS_ABORTED:
                if self.task_status.task_type == self.TASK_CIRCLE:
                    self.get_logger().warn(f'Navigation circle to goal {self.current_index} failed')
                    self.__CirclePathPoint()
                else:
                    self.task_status.status = self.TASK_FAILED
                    self.get_logger().warn(f'Navigation point to goal {self.current_index} failed')
                return
            elif goal_status == GoalStatus.STATUS_CANCELED:
                self.task_status.status = self.TASK_CANCEL
                self.get_logger().info(f'Navigation to goal {self.current_index} canceled')
                return
            elif goal_status == GoalStatus.STATUS_SUCCEEDED:
                if self.task_status.task_type == self.TASK_POINT:
                    self.task_status.status = self.TASK_SUCCESS
                    self.get_logger().info('Completed all nav pose actions')
                else:
                    self.__CirclePathPoint()
            else:
                if self.task_status.task_type == self.TASK_CIRCLE:
                    self.get_logger().warn(f'Unknown circle goal status {goal_status} failed')
                    self.__CirclePathPoint()
                else:
                    self.task_status.status = self.TASK_FAILED
                    self.get_logger().warn(f'Unknown goal status: {goal_status}')


    # ------------------ 取消目标 ------------------
    def cancel_all_goals(self):
        if self._goal_handle is not None and self._goal_handle.accepted:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Cancelled current goal'))
            self.get_logger().info('Cancelling current goal...')
        # with self.status_lock:
        #     self.task_status.status = self.TASK_CANCEL
        self._goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        executor.shutdown()
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
        node.destroy_node()

if __name__ == "__main__":
    main()
