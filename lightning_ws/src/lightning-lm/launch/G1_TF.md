# G1 localization TF

`g1_mapping.launch.py` and `g1_localization.launch.py` include
`g1_online.launch.py`, which loads `share/lightning/urdf/g1.urdf` into
robot_state_publisher. URDF is not used to overwrite the algorithm's IMU
extrinsics or navigation settings.

Localization's ROS adapter (`LocSystem`) interprets the algorithm result as
T_map_lidar and publishes T_map_base = T_map_lidar * inverse(T_base_lidar).
It reads only `/tf_static` (reliable, transient-local), resolves the static
base_link-to-mid360_link chain, and caches its inverse once. Before a valid
extrinsic arrives it drops TF output with a throttled warning. The high-rate
callback does not perform TF lookup. Restart localization after changing URDF.
The core ToGeoMsg, LIO, and PGO implementations are unchanged.

Expected localization tree: map -> base_link -> mid360_link, with the other
URDF fixed links retained. Do not run another map-to-base publisher in parallel.
Mapping launch restores the original online mapping entry; this change does
not add dynamic mapping TF to the original SlamSystem.

After rebuilding and sourcing the workspace:

```bash
ros2 launch lightning g1_localization.launch.py map_path:=/absolute/path/to/lightning_map
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 run tf2_ros tf2_echo map base_link
```

Expected static translation: (0.0002835, 0.00003, 1.30618), pitch 0.0401425728 rad.
The dynamic message preserves the localization timestamp and map origin. Map z=0
is not necessarily the physical floor. This adapter does not rebase saved maps.
`start_rviz:=true` opens default RViz; pass `rviz_config:=...` for a saved display.
Optional floor/obstacle-height launch overrides are empty by default, preserving YAML.

Validation: `python3 -m unittest discover -s test -p test_g1_extrinsic.py` checks
the URDF transform direction with zero and nonzero poses, and launch syntax.
It does not certify live localization accuracy or the measured mounting geometry.
