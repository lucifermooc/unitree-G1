import math
import unittest
from unitree_go.msg import SportModeState
from g1_nav_bridge.sport_to_odom import convert_state


class ConversionTests(unittest.TestCase):
    def sample(self):
        m = SportModeState()
        m.stamp.sec, m.stamp.nanosec = 1789027818, 107790946
        m.imu_state.quaternion = [-1.0, 0.0, 0.0, 0.0]
        m.position = [1.0, 2.0, 0.77]
        m.body_height = 0.77
        m.velocity = [0.2, -0.1, 0.03]
        m.yaw_speed = 0.15
        return m

    def test_fields_and_negative_quaternion(self):
        m = self.sample()
        out = convert_state(m)
        self.assertEqual(out.header.stamp.sec, m.stamp.sec)
        self.assertEqual(out.header.stamp.nanosec, m.stamp.nanosec)
        self.assertEqual((out.header.frame_id, out.child_frame_id), ('odom', 'base_link'))
        self.assertEqual(out.pose.pose.position.z, 0.0)
        self.assertEqual(out.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(out.pose.pose.orientation.w, 1.0)
        self.assertAlmostEqual(out.twist.twist.linear.x, 0.2)
        self.assertAlmostEqual(out.twist.twist.angular.z, 0.15)

    def test_yaw_order(self):
        m = self.sample()
        m.imu_state.quaternion = [math.sqrt(.5), 0.0, 0.0, math.sqrt(.5)]
        out = convert_state(m)
        self.assertAlmostEqual(out.pose.pose.orientation.z, math.sqrt(.5))
        self.assertAlmostEqual(out.pose.pose.orientation.w, math.sqrt(.5))

    def test_invalid(self):
        for kind in ('error', 'zero_quat', 'nan', 'stamp'):
            m = self.sample()
            if kind == 'error': m.error_code = 1
            if kind == 'zero_quat': m.imu_state.quaternion = [0.0]*4
            if kind == 'nan': m.velocity = [float('nan'), 0.0, 0.0]
            if kind == 'stamp': m.stamp.sec = 0; m.stamp.nanosec = 0
            with self.assertRaises(ValueError): convert_state(m)
