"""loc_guard 配置契约：阈值来自 2026-09-29 走廊漂移实测，改动需同时写明实测理由。"""
from pathlib import Path
import re
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]


class LocGuardConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT/'config/default_livox.yaml').read_text())
        self.guard = self.config['loc_guard']

    def test_enabled_freeze_with_rollback(self):
        self.assertTrue(self.guard['enabled'])
        # 2026-09-30 新图导航实测拉偏 1.5~2.2 m 且守护每次判对：改为冻结 + 退回（用户要求直接实机测试）
        self.assertEqual(self.guard['action'], 'freeze')
        self.assertTrue(self.guard['rollback'])

    def test_drift_threshold_between_normal_and_incident(self):
        # 09-30 手动慢走窗口内修正 <= 0.18 m；09-29 出事时 0.64 m，09-30 导航几秒内就过 0.2 m
        self.assertGreater(self.guard['max_drift_m'], 0.18)
        self.assertLess(self.guard['max_drift_m'], 0.64)
        self.assertLessEqual(self.guard['max_drift_deg'], 10.0)

    def test_static_threshold_catches_slow_creep(self):
        # 09-30 14:51 静止时 40 s 被拉走 0.19 m；静止时 LIO 20 min 只漂 ~5 cm
        self.assertLess(self.guard['max_drift_static_m'], 0.19)
        self.assertGreater(self.guard['max_drift_static_m'], 0.05)
        self.assertLess(self.guard['max_drift_static_m'], self.guard['max_drift_m'])
        self.assertGreaterEqual(self.guard['window_sec'], 40.0)

    def test_odom_tolerance_covers_measured_error(self):
        # 09-29 腿式里程计实测：误差/行程 p95 10.9%、航向 p95 6°；转身时单帧误差更大，需要连续多帧才改判
        self.assertGreaterEqual(self.guard['odom_tol_ratio'], 0.11)
        self.assertGreaterEqual(self.guard['odom_tol_deg'], 6.0)
        self.assertGreaterEqual(self.guard['odom_persist_frames'], 3)

    def test_window_and_recovery_sane(self):
        self.assertGreater(self.guard['window_sec'], 0)
        self.assertGreater(self.guard['window_dist'], 0)
        self.assertGreaterEqual(self.guard['recover_frames'], 3)
        self.assertLess(self.guard['recover_res_m'], self.guard['max_drift_m'])
        self.assertGreater(self.guard['reloc_hint_frames'], self.guard['recover_frames'])

    def test_odom_is_sport_to_odom_output(self):
        bridge = yaml.safe_load((ROOT.parent/'g1_nav_bridge/config/nav_bridge.yaml').read_text())
        self.assertEqual(self.guard['odom_topic'], bridge['g1_sport_to_odom']['ros__parameters']['output_topic'])

    def test_init_yaw_search_is_narrow(self):
        # 上游全范围 YawSearch 会选错角度（issue #123），只允许在人工给的航向附近搜
        self.assertLessEqual(self.guard['init_yaw_search_deg'], 30.0)
        self.assertGreater(self.guard['init_yaw_search_step_deg'], 0)

    def test_ndt_jump_gate(self):
        # 单帧跳变检查：要比 LIO 单帧误差（厘米级）大得多，又要挡住导航时 0.3~0.6 m 的错配
        self.assertGreater(self.guard['max_ndt_jump_m'], 0.05)
        self.assertLessEqual(self.guard['max_ndt_jump_m'], 0.3)
        self.assertLessEqual(self.guard['max_ndt_jump_deg'], 5.0)
        self.assertGreater(self.guard['max_ndt_jump_deg'], 0.5)

    def test_loc_input_keys_are_read(self):
        # 2026-09-30 投影关键帧 A/B 开关：默认保持上游行为（带历史关键帧投影）
        loc_input = self.config['loc_input']
        self.assertTrue(loc_input['proj_kfs'])
        source = (ROOT/'src/core/localization/localization.cpp').read_text()
        read = set(re.findall(r'GetOptional\("loc_input", "(\w+)"', source))
        self.assertEqual(set(loc_input), read)

    def test_every_key_is_read_by_lidar_loc(self):
        source = (ROOT/'src/core/localization/lidar_loc/lidar_loc.cc').read_text()
        source += (ROOT/'src/core/system/loc_system.cc').read_text()
        read = set(re.findall(r'GetOptional\("loc_guard", "(\w+)"', source))
        self.assertEqual(set(self.guard), read)


if __name__ == '__main__':
    unittest.main()
