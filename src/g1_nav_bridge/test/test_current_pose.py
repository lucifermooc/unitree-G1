import unittest
from types import SimpleNamespace
from geometry_msgs.msg import TransformStamped
from g1_nav_bridge.tf_to_current_pose import to_pose, TfToCurrentPose


class CurrentPoseTests(unittest.TestCase):
    def transform(self):
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.header.stamp.sec = 100
        t.transform.translation.x = 2.
        t.transform.translation.z = -1.2
        t.transform.rotation.w = 1.
        return t

    def test_full_pose_and_source_timestamp(self):
        p = to_pose(self.transform())
        self.assertEqual(p.header.frame_id, 'map')
        self.assertEqual(p.header.stamp.sec, 100)
        self.assertEqual(p.pose.position.z, -1.2)
        self.assertEqual(p.pose.position.x, 2.)
        self.assertFalse(hasattr(p, 'child_frame_id'))

    def test_invalid_rejected(self):
        t = self.transform()
        t.transform.translation.x = float('nan')
        with self.assertRaises(ValueError):
            to_pose(t)

    def test_stale_and_duplicate_not_published(self):
        sent = []
        t = self.transform()
        now = SimpleNamespace(nanoseconds=100_100_000_000)
        node = SimpleNamespace(global_frame='map', base_frame='base_link', max_age=.5,
            last_stamp=None, buffer=SimpleNamespace(lookup_transform=lambda *args: t),
            get_clock=lambda: SimpleNamespace(now=lambda: now),
            get_logger=lambda: SimpleNamespace(warning=lambda *a, **kw: None),
            publisher=SimpleNamespace(publish=sent.append))
        TfToCurrentPose.publish_pose(node)
        TfToCurrentPose.publish_pose(node)
        self.assertEqual(len(sent), 1)
        t.header.stamp.sec = 99
        TfToCurrentPose.publish_pose(node)
        self.assertEqual(len(sent), 1)
