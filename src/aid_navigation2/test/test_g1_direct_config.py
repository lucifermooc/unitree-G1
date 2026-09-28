"""参数不再由 launch 计算：这些用例直接校验 yaml 里的运行值，外加两个删层开关。"""
import ast
import copy
import importlib.util
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
LAUNCH = ROOT / 'launch/g1_navigation_direct.launch.py'
ROBOT_LAUNCH = SRC / 'robot_bringup/launch/robot.launch.py'
NAV_BRIDGE_LAUNCH = SRC / 'g1_nav_bridge/launch/g1_navigation.launch.py'
FILTER_CFG = SRC / 'g1_nav_bridge/config/d435_mark_filter.yaml'
POINT_FILTER_CFG = SRC / 'point_filter/config/point_filter.yaml'
spec = importlib.util.spec_from_file_location('g1_direct', LAUNCH)
direct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(direct)


class G1ParamTests(unittest.TestCase):
    def setUp(self):
        self.nav = yaml.safe_load((ROOT / 'param/nav2_params.yaml').read_text())
        self.safe = yaml.safe_load(
            (SRC / 'robot_bringup/param/collision_monitor_params.yaml').read_text())
        self.local = self.nav['local_costmap']['local_costmap']['ros__parameters']
        self.globl = self.nav['global_costmap']['global_costmap']['ros__parameters']

    def test_direct_frames(self):
        for p in (self.local, self.globl):
            self.assertEqual((p['global_frame'], p['robot_base_frame']), ('map', 'base_link'))
        b = self.nav['behavior_server']['ros__parameters']
        self.assertEqual((b['local_frame'], b['global_frame']), ('map', 'map'))
        c = self.safe['collision_monitor']['ros__parameters']
        self.assertEqual((c['base_frame_id'], c['odom_frame_id']), ('base_link', 'map'))
        self.assertFalse(c['base_shift_correction'])
        self.assertNotIn('amcl', self.nav)

    def test_lidar_source_is_registered_scan(self):
        """MID360 在两张图的 mid360_voxel_layer（STVL）里：吃 LIO 去畸变单帧经近距剔除后的点云，标记/清除分源。"""
        for p in (self.local, self.globl):
            self.assertNotIn('obstacle_layer', p['plugins'])   # 2D raytrace 清不掉头部盲区里的格子
            self.assertNotIn('obstacle_layer', p)
            stvl = p['mid360_voxel_layer']
            self.assertEqual(stvl['plugin'], 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
            self.assertEqual(stvl['observation_sources'].split(), ['livox_mark', 'livox_clear'])
            # 同原 obstacle_layer.footprint_clearing_enabled：贴着家具走时，半径内的 MID360 格不能留成致命格
            self.assertTrue(stvl['update_footprint_enabled'])
            mark, clear = stvl['livox_mark'], stvl['livox_clear']
            for src in (mark, clear):
                # STVL 没有 obstacle_min_range：必须吃剔除过机身回波的话题，不能直接吃 registered_scan
                self.assertEqual(src['topic'], direct.MID360_NAV)
                self.assertEqual(src['data_type'], 'PointCloud2')
                self.assertNotIn('sensor_frame', src)            # 用消息自带的雷达系 + 扫描时刻
            self.assertTrue(mark['marking']); self.assertFalse(mark['clearing'])
            self.assertFalse(clear['marking']); self.assertTrue(clear['clearing'])
            # 地面 z=0；0.10 时 1.7~2.1 m 处 0.10~0.13 m 的地面回波被标成单格噪点（2026-09-24 实机）
            self.assertAlmostEqual(mark['min_obstacle_height'], 0.15)
            self.assertAlmostEqual(mark['max_obstacle_height'], 1.8)
            self.assertAlmostEqual(mark['obstacle_range'], 2.5)
            self.assertTrue(mark['clear_after_reading'])
            # 3D 雷达视锥：水平 360°，清除半径覆盖标记半径，最近距离不小于剔除半径
            self.assertEqual(clear['model_type'], 1)
            self.assertGreater(clear['horizontal_fov_angle'], 6.27)
            self.assertGreaterEqual(clear['min_z'], direct.MID360_MIN_RANGE)
            self.assertGreater(clear['max_z'], mark['obstacle_range'])
            # 对称视锥半角必须覆盖倒装后的仰角下缘
            self.assertGreater(clear['vertical_fov_angle'] / 2, 0.882)  # 实测雷达系仰角下限 -50.5°
            # 非重复扫描：加速度过大会把没被每帧打到的真障碍提前清掉
            self.assertLessEqual(clear['decay_acceleration'], 2.0)
            # 标记源的缓冲至少覆盖一个更新周期，否则两次更新之间的帧被丢掉（global 1 Hz 时尤其要紧）
            self.assertGreaterEqual(mark['observation_persistence'] * p['update_frequency'], 1.0 - 1e-9)
        self.assertAlmostEqual(direct.MID360_MIN_RANGE, 0.25)
        self.assertEqual(direct.MID360_RAW, '/lightning/registered_scan')
        text = LAUNCH.read_text()
        self.assertIn("executable='scan_range_filter'", text)
        self.assertIn("'min_range': MID360_MIN_RANGE", text)
        # /livox/points（point_filter）去机身回波的盲区必须与 Nav2 一致，否则 RViz 里还能看到机身
        pf = yaml.safe_load(POINT_FILTER_CFG.read_text())['point_filter']['ros__parameters']
        self.assertAlmostEqual(pf['blind'], direct.MID360_MIN_RANGE)
        # 足迹内的点必须在进 STVL 前删掉：自清只遮当前足迹，走开后体素会露出来
        self.assertIn("'crop_radius': MID360_CROP_RADIUS", text)
        for p in (self.local, self.globl):
            self.assertAlmostEqual(direct.MID360_CROP_RADIUS, p['robot_radius'])
        c = self.safe['collision_monitor']['ros__parameters']['pointcloud1']
        self.assertEqual(c['topic'], '/lightning/registered_scan')
        self.assertAlmostEqual(c['min_height'], 0.10)

    def test_collision_geometry_is_consistent(self):
        """碰撞判定只看 robot_radius；inflation 仅决定代价梯度长度。"""
        for p in (self.local, self.globl):
            self.assertAlmostEqual(p['robot_radius'], 0.50)
            inf = p['inflation_layer']
            band = inf['inflation_radius'] - p['robot_radius']
            self.assertGreater(band, 0.10, '膨胀半径必须大于碰撞半径，否则边界出现代价断崖')
            self.assertLess(band, 0.40, '梯度带过宽会把大片区域染成中高代价')
        self.assertAlmostEqual(self.local['inflation_layer']['cost_scaling_factor'], 8.0)
        self.assertAlmostEqual(self.globl['inflation_layer']['cost_scaling_factor'], 6.0)
        # 提速后制动距离必须小于传感器盲区量级
        sm = self.nav['velocity_smoother']['ros__parameters']
        brake = sm['max_velocity'][0]**2 / (2*abs(sm['max_decel'][0]))
        self.assertLess(brake, 0.45, f'制动距离 {brake:.2f}m 超过盲区 0.40m')
        zone = self.safe['collision_monitor']['ros__parameters']['SafetyZone']
        self.assertEqual(zone['action_type'], 'stop')
        self.assertAlmostEqual(zone['radius'], 0.70)  # robot_radius + 0.20

    def test_stvl_d435_sources(self):
        for p in (self.local, self.globl):
            stvl = p['stvl_voxel_layer']
            self.assertEqual(stvl['plugin'], 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
            self.assertEqual(stvl['observation_sources'].split(),
                             ['realsense_mark', 'realsense_mark_tall', 'realsense_clear'])
            # 自清必须关：半径 = robot_radius，会把半径内的真实障碍当自身点抹掉（正前方靠 D435 在接触前停住）
            self.assertFalse(stvl['update_footprint_enabled'])
            # 高物体源：下限必须高于右前方稳定错深度簇（0.37~0.59 m），否则把假点放回 costmap
            tall = stvl['realsense_mark_tall']
            # 孤立飞点高 ~1.0 m，高物体源也会标它：两个标记源都必须吃剔除飞点后的点云
            self.assertEqual(tall['topic'], direct.FILTERED_D435)
            self.assertTrue(tall['marking']); self.assertFalse(tall['clearing'])
            self.assertGreaterEqual(tall['min_obstacle_height'], 0.65)
            self.assertLessEqual(tall['obstacle_range'], 2.5)
            mark = stvl['realsense_mark']
            # 清除必须始终吃原始点云（否则被删区域没有视锥清除）
            self.assertEqual(mark['topic'], direct.FILTERED_D435)
            self.assertEqual(stvl['realsense_clear']['topic'], direct.RAW_D435)
            self.assertNotIn('sensor_frame', mark)
            # 离相机 3D 距离（地上矮障碍水平只到 ~1.1 m）。2.0 实测把顶部几行的掠射错深度标进来，悬空剔除也压不干净
            self.assertAlmostEqual(mark['obstacle_range'], 1.5)
            # 试过 0.05 后退回：D435 点云地面偏低 ~4.5 cm 且随站姿变
            self.assertAlmostEqual(mark['min_obstacle_height'], 0.15)
            self.assertEqual(mark['voxel_min_points'], 4)
        # 全局记忆不短于局部
        for layer in ('stvl_voxel_layer', 'mid360_voxel_layer'):
            self.assertGreaterEqual(self.globl[layer]['voxel_decay'], self.local[layer]['voxel_decay'])

    def test_mark_filter_toggle(self):
        text = LAUNCH.read_text()
        # 默认开：原始点云里的孤立飞点会在 base_link 旁边标出挂 voxel_decay 的致命格
        self.assertIn("DeclareLaunchArgument('use_d435_mark_filter', default_value='true'", text)
        for f in (ROBOT_LAUNCH, NAV_BRIDGE_LAUNCH):   # 上层 launch 的默认值才是实际生效的
            self.assertRegex(f.read_text(), r"'use_d435_mark_filter', default_value='true'")
        nav = copy.deepcopy(self.nav)
        self.assertTrue(direct.raw_mark(nav))
        for name in direct.COSTMAPS:
            stvl = nav[name][name]['ros__parameters']['stvl_voxel_layer']
            for src in direct.MARK_SOURCES:
                self.assertEqual(stvl[src]['topic'], direct.RAW_D435)
            self.assertAlmostEqual(stvl['realsense_mark']['obstacle_range'], 1.5)  # 开关不联动改距离
            self.assertEqual(stvl['realsense_clear']['topic'], direct.RAW_D435)
        self.assertFalse(direct.raw_mark(nav))  # 幂等
        # 只开 4 邻域：帧间一致性会额外删掉 20~30% 的真实物体
        f = yaml.safe_load(FILTER_CFG.read_text())['d435_mark_filter']['ros__parameters']
        self.assertTrue(f['neighbor_filter'])
        # realsense_mark 标到 1.5 m 以外时，必须开悬空点剔除挡掠射地面的错深度（只动离相机 >min_range 的点）
        if self.local['stvl_voxel_layer']['realsense_mark']['obstacle_range'] > 1.5:
            self.assertTrue(f['floating_filter'])
            self.assertLessEqual(f['min_range'], 1.5)
        self.assertGreaterEqual(f['min_depth'], 0.35)   # 镜头前 0.2~0.3 m 的小团点 4 邻域删不掉
        # 走路时前摆的手（实测离 base_link 水平 ≤0.48 m、高 0.57~0.73 m）必须删；高度带不能吃掉腿和躯干，
        # 半径不超过足迹（正前方只有 D435，闯进足迹的人要靠它停下）
        self.assertGreaterEqual(f['self_crop_radius'], 0.48)
        self.assertLessEqual(f['self_crop_radius'], self.local['robot_radius'])
        self.assertLessEqual(f['self_crop_z_min'], 0.57)
        self.assertGreaterEqual(f['self_crop_z_max'], 0.73)
        self.assertGreaterEqual(f['self_crop_z_min'], 0.40)
        self.assertLessEqual(f['self_crop_z_max'], 0.90)
        self.assertFalse(f['temporal_filter'])
        self.assertEqual(f['output_topic'], direct.FILTERED_D435)

    def test_d435_toggle_keeps_mid360(self):
        """use_realsense_obstacles:=false 摘掉的 stvl_voxel_layer 必须只有 D435，MID360 层原样保留。"""
        text = LAUNCH.read_text()
        self.assertIn("([] if use_d435 else ['stvl_voxel_layer'])", text)
        nav = copy.deepcopy(self.nav)
        self.assertTrue(direct.drop_layers(nav, ['stvl_voxel_layer']))
        for name in direct.COSTMAPS:
            p = nav[name][name]['ros__parameters']
            self.assertIn('mid360_voxel_layer', p['plugins'])
            self.assertNotIn('livox', yaml.safe_dump(self.nav[name][name]['ros__parameters']['stvl_voxel_layer']))

    def test_costmap_rates_match_local_control(self):
        self.assertEqual((self.local['update_frequency'], self.local['publish_frequency']), (10.0, 5.0))
        self.assertEqual((self.globl['update_frequency'], self.globl['publish_frequency']), (5.0, 5.0))
        # 标记源缓存只攒一个更新周期：更长会让人走开后的障碍多挂这么久
        mark = self.globl['mid360_voxel_layer']['livox_mark']
        self.assertAlmostEqual(mark['observation_persistence'] * self.globl['update_frequency'], 1.0)
        self.assertTrue(self.local['rolling_window'])

    def test_jazzy_plugin_names(self):
        p = self.nav['planner_server']['ros__parameters']
        self.assertEqual(p['GridBased']['plugin'], 'nav2_smac_planner::SmacPlanner2D')
        b = self.nav['behavior_server']['ros__parameters']
        for key in b['behavior_plugins']:
            self.assertTrue(b[key]['plugin'].startswith('nav2_behaviors::'), b[key]['plugin'])
        bt = self.nav['bt_navigator']['ros__parameters']
        self.assertNotIn('plugin_lib_names', bt)    # jazzy 用默认全集
        c = self.nav['controller_server']['ros__parameters']
        self.assertEqual(c['progress_checker_plugins'], ['progress_checker'])
        self.assertNotIn('progress_checker_plugin', c)

    def test_safe_topics_open_loop_limits(self):
        for node in ('controller_server', 'bt_navigator'):
            self.assertEqual(self.nav[node]['ros__parameters']['odom_topic'], '/odom')
        s = self.nav['velocity_smoother']['ros__parameters']
        self.assertEqual(s['feedback'], 'OPEN_LOOP')
        self.assertNotIn('odom_topic', s)
        self.assertEqual(s['max_velocity'], [0.45, 0.0, 0.9])
        self.assertEqual(s['min_velocity'], [0.0, 0.0, -0.9])
        self.assertEqual(s['deadband_velocity'], [0.0, 0.0, 0.0])
        c = self.safe['collision_monitor']['ros__parameters']
        self.assertEqual((c['cmd_vel_in_topic'], c['cmd_vel_out_topic']), ('/cmd_vel', '/cmd_vel_safe'))
        configs = list((SRC / 'g1_nav_bridge/config').glob('*.yaml'))
        bridge = next(yaml.safe_load(p.read_text())['g1_cmdvel_to_sport']['ros__parameters']
                      for p in configs if 'g1_cmdvel_to_sport' in yaml.safe_load(p.read_text()))
        self.assertEqual(bridge['cmd_vel_topic'], '/cmd_vel_safe')
        self.assertGreater(bridge['duration'], 0.0)

    def test_obstacle_critic_matches_local_inflation(self):
        """ObstaclesCritic 复算代价，几何必须与 local_costmap 的 inflation 层一致。"""
        f = self.nav['controller_server']['ros__parameters']['FollowPath']
        oc = f['ObstaclesCritic']
        inf = self.local['inflation_layer']
        self.assertTrue(oc['enabled'])
        self.assertAlmostEqual(oc['inflation_radius'], inf['inflation_radius'])
        self.assertAlmostEqual(oc['cost_scaling_factor'], inf['cost_scaling_factor'])
        self.assertTrue(f['CostCritic']['enabled'])
        self.assertLess(f['CostCritic']['cost_weight'], 8.0)   # 两个 critic 都开时须退为配角
        self.assertAlmostEqual(
            self.nav['controller_server']['ros__parameters']['failure_tolerance'], 0.3)

    def test_controller_envelope(self):
        p = self.nav['controller_server']['ros__parameters']
        self.assertAlmostEqual(p['FollowPath']['model_dt'], 1. / p['controller_frequency'])
        self.assertEqual(p['FollowPath']['vx_min'], 0.0)   # 不后退
        self.assertEqual(p['FollowPath']['vx_max'], 0.45)
        self.assertEqual(p['FollowPath']['wz_max'], 0.9)
        b = self.nav['behavior_server']['ros__parameters']
        self.assertEqual((b['max_rotational_vel'], b['min_rotational_vel']), (0.9, 0.8))

    def test_never_reverse(self):
        """G1 收到负的前进速度会摔倒：三道防线都不得放宽。"""
        f = self.nav['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['vx_min'], 0.0)
        s = self.nav['velocity_smoother']['ros__parameters']
        self.assertGreaterEqual(s['min_velocity'][0], 0.0)
        bt = (ROOT / 'behavior_trees' /
              'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml').read_text()
        self.assertNotIn('<BackUp', bt)      # 恢复行为不得倒车
        bridge = (SRC / 'g1_nav_bridge/src/cmdvel_to_sport.cpp').read_text()
        self.assertIn('velocity[0] = 0.0F;', bridge)   # 桥接层兜底钳位

    def test_keepout_layer_present_by_default(self):
        for p in (self.local, self.globl):
            self.assertIn('keepout_layer', p['plugins'])
            self.assertEqual(p['keepout_layer']['plugin'], 'aid_costmap_plugin/KeepoutLayer')

    def test_drop_layers_toggle(self):
        nav = yaml.safe_load((ROOT / 'param/nav2_params.yaml').read_text())
        self.assertTrue(direct.drop_layers(nav, ['keepout_layer', 'stvl_voxel_layer']))
        for name in ('local_costmap', 'global_costmap'):
            p = nav[name][name]['ros__parameters']
            self.assertNotIn('keepout_layer', p['plugins'])
            self.assertNotIn('keepout_layer', p)
            self.assertNotIn('stvl_voxel_layer', p['plugins'])
            self.assertIn('inflation_layer', p['plugins'])   # 其余层不受影响
        self.assertFalse(direct.drop_layers(nav, ['keepout_layer']))  # 幂等

    def test_launch_no_longer_injects_parameters(self):
        for p in (ROOT / 'launch').glob('*.py'):
            ast.parse(p.read_text())
        self.assertFalse((ROOT / 'launch/g1_config.py').exists())
        text = LAUNCH.read_text()
        for gone in ('floor_z', 'base_floor_z', 'cloud_topic', 'sensor_frame',
                     'robot_radius', 'realsense_topic', 'static_transform_publisher',
                     "package='lightning'"):
            self.assertNotIn(gone, text)
        bringup = (SRC / 'robot_bringup/launch/robot.launch.py').read_text()
        self.assertNotIn("LaunchConfiguration('robot_radius')", bringup)
        self.assertNotIn("LaunchConfiguration('floor_z')", bringup)

    def test_reference_isolation_and_map_resolution(self):
        self.assertFalse((SRC / 'g1_nav_integration').exists())
        source = (SRC / 'lightning-lm/src/core/system/slam.cc').read_text()
        self.assertIn('YAML::Key << "resolution" << YAML::Value << map.info.resolution', source)
        self.assertNotIn('YAML::Key << "resolution" << YAML::Value << float(0.05)', source)


if __name__ == '__main__':
    unittest.main(verbosity=2)
