"""Run with python3 -m unittest discover -s test -p test_g1_extrinsic.py.

Numerical contract tests (not a substitute for live ROS TF testing).
"""
import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                  [-sp, cp*sr, cp*cr]]
    out[:3, 3] = xyz
    return out


class ExtrinsicContractTest(unittest.TestCase):
    def test_urdf_direction(self):
        root = ET.parse(ROOT / 'urdf/g1.urdf').getroot()
        joint = root.find("./joint[@name='base_to_mid360_joint']")
        self.assertEqual(joint.find('parent').get('link'), 'base_link')
        self.assertEqual(joint.find('child').get('link'), 'mid360_link')
        origin = joint.find('origin')
        base_lidar = transform(*[list(map(float, origin.get(k).split())) for k in ('xyz', 'rpy')])
        for xyz, rpy in [([0, 0, 0], [0, 0, 0]), ([2, -3, .1], [.1, .2, 1.3])]:
            map_base = transform(xyz, rpy)
            map_lidar = map_base @ base_lidar
            np.testing.assert_allclose(map_lidar @ np.linalg.inv(base_lidar), map_base, atol=1e-12)

    def test_launch_syntax(self):
        files = list((ROOT / 'launch').glob('*.launch.py'))
        self.assertEqual(len(files), 3)
        for path in files:
            ast.parse(path.read_text())


if __name__ == '__main__':
    unittest.main()
