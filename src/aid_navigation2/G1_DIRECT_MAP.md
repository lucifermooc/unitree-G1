# G1 direct-map navigation profile

This entry reuses the existing planner/BT, static map, keepout layer and frontend
interfaces. It does not import the reference package's URDF, pelvis bridge,
joint-state reader, pose publishers or velocity estimator. The reference folder
is COLCON_IGNORE'd to avoid duplicate g1_nav_bridge packages; its files remain.

## Entry and prerequisites

Run the existing driver, PointCloud2 converter and original Lightning localization
separately. This launch does not start/restart them or publish any TF. Confirm:

- Localization publishes valid, current map -> base_link.
- The actual PointCloud2 header frame is connected to base_link with the correct
  transform. A renamed frame is not a numerical point transformation. In original
  Lightning, base_link naming alone does not imply the G1 torso reference point.
- Both streams use synchronized timestamps.
- A saved map YAML exists. No live mapping /map publisher should compete with
  the navigation map_server.
- Supply measured ground Z in map AND in base_link. These are different frames.
  Do not copy g2p5.floor_height blindly: it is evaluated in local keyframe clouds.

```bash
ros2 launch aid_navigation2 g1_navigation_direct.launch.py \
  map:=/absolute/path/map.yaml \
  floor_z:=<measured-map-ground-z> \
  base_floor_z:=<measured-base-link-ground-z>
```

Replace placeholders with numbers. cloud_topic defaults to /livox/points and
sensor_frame to mid360_link. robot_radius=0.40 is an initial test envelope, not
a certified G1 footprint. Re-measure with arms/payload and gait. The configured
ground heights assume the test stance and approximately level floor; slopes,
tilt and body height changes require further verification.

This entry owns Nav2, map_server and collision_monitor. Do not also start the
old navigation2.launch.py or another collision monitor. If the existing
robot_bringup robot.launch.py provides the driver/frontend/command bridge, use
start_collision_monitor:=false there. Do not ask the frontend launch manager
to start a second navigation stack. Its automatic navigation entry has NOT been
switched to this profile: measured heights must first be supplied and validated.

## Preserved safety chain

controller/behaviors -> /cmd_vel_nav -> OPEN_LOOP velocity smoother -> /cmd_vel
-> collision_monitor -> /cmd_vel_safe -> existing G1 command bridge.

The command bridge directly forwards each fresh Twist through Unitree API 7105.
Stopping and mode gating must happen upstream; the robot also expires each command
after the configured short duration.
The collision monitor now uses a conservative fixed stop circle; missing/stale
clouds must be checked during commissioning. Four points trigger the zone: this
does not guarantee detection of sparse objects. No claims about certified safety.

The profile avoids odom TF lookups (including behavior local_frame and collision
monitor odom_frame_id, both map). It DOES NOT prove MPPI/BT need no velocity data.
Their existing odom-topic settings remain visible; no fake zero or globally
differentiated odometry is generated. Verify their runtime behavior and actual
velocity input before enabling motion. OPEN_LOOP only removes the smoother's
closed-loop velocity dependency, not every controller's dependency.

## Acceptance

Generated nav.yaml/collision.yaml are kept under /tmp/g1_direct_nav_* for review.
First keep drive disabled: place and remove a known box; check both costmaps and
collision monitor. With ground=-1.2 in map, a box point at -0.9 must be eligible
while ground itself is excluded. This profile uses [ground+0.10, ground+1.8].
Objects below 10 cm, sensor blind spots and body returns are not solved by this
configuration. Check localization jumps before any controlled low-speed test.

The only Lightning C++ change uses map.info.resolution when saving map.yaml.
It does not rewrite old map files; rebuild Lightning for future saves. Use new
map names because upstream's same-name save can replace existing map directories.

```bash
python3 src/aid_navigation2/test/test_g1_direct_config.py
```

Tests cover parameter/geometry contracts, not ROS activation or hardware motion.
Legacy parameter files remain unmodified so the new entry is opt-in.
