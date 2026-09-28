# Unitree planar odometry for Nav2

`sport_to_odom` converts `/odommodestate` (`unitree_go/msg/SportModeState`)
to `/odom` (`nav_msgs/msg/Odometry`). It does not broadcast TF or send
motion commands. The existing nav_bridge launch starts it alongside the
command bridge.

The output pose is boot-relative XY/yaw in `odom`, not Lightning `map`.
Child frame is the existing ground-projected `base_link`. This is a planar
approximation, not a full 6-DoF torso-to-ground kinematic conversion: preserve
source XY, set Z/roll/pitch to zero; use source body XY velocity and yaw_speed.
Check direction on real hardware, particularly with substantial torso tilt.
Unitree quaternion w,x,y,z is normalized and converted to ROS yaw quaternion.
Source timestamp is preserved. Invalid/error and nonincreasing messages are
dropped. Restart the adapter on a robot clock reset; it never hides a stale
stream by repeatedly publishing old data or zero velocity.

Covariances in nav_bridge.yaml are provisional variances, not measured values.
Do not fuse this output into an estimator without validating frame, time and
uncertainty. `body_height` is not added to position.z. The supplied stationary
sample does not establish absolute position-height semantics.

The G1 direct-map Nav2 profile consumes `/odom` in controller_server and
bt_navigator. Velocity smoother remains OPEN_LOOP. No `odom` TF is
required by these velocity consumers; this is not a general replacement for
the standard map->odom->base TF architecture for other applications.

After building and sourcing Unitree ROS messages and this workspace:

```bash
ros2 run g1_nav_bridge sport_to_odom
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Do not start this standalone node again if nav_bridge.launch.py already owns it.
Check live ROS/source clock synchronization, forward/turn velocity signs and
rate before navigation. Stream-loss warnings alone are NOT an automatic stop
interlock; loss of odometry still needs a tested navigation safety response.
Map/localization TF, obstacle observations, footprints, costmaps and command
bridge validation remain necessary. This change alone does not certify navigation.
