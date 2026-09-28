"""Configuration contract tests; not a substitute for bag/real-robot validation."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


class G2P5ConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT/'config/default_livox.yaml').read_text())

    def test_nominal_height_and_fallback(self):
        root = ET.parse(ROOT/'urdf/g1.urdf').getroot()
        origin = root.find("./joint[@name='base_to_mid360_joint']/origin")
        height = float(origin.get('xyz').split()[2])
        g = self.config['g2p5']
        self.assertAlmostEqual(g['lidar_height'], height)
        self.assertAlmostEqual(g['floor_height'], -height)
        self.assertTrue(g['esti_floor'])

    def test_floor_and_one_meter_obstacle_survive_input_crop(self):
        g, roi = self.config['g2p5'], self.config['roi']
        for z in (g['floor_height'], g['floor_height'] + 1.0):
            self.assertLess(roi['height_min'], z)
            self.assertGreater(roi['height_max'], z)
        self.assertLess(g['min_th_floor'], 1.0)
        self.assertGreater(g['max_th_floor'], 1.0)
        self.assertEqual(g['grid_map_resolution'], 0.05)

    def test_frontend_and_extrinsics_unchanged(self):
        before = yaml.safe_load((ROOT/'config/default_livox.yaml.before_g2p5_20260910').read_text())
        for key in ('common', 'fasterlio', 'system', 'loop_closing', 'lidar_loc', 'pgo'):
            self.assertEqual(before[key], self.config[key])


if __name__ == '__main__':
    unittest.main()
