# G1 workspace integration

The replacement driver rotates both points and IMU vectors using the configured
extrinsic rotation. G1_MID360s_config.json uses roll=180 degrees with zero
translation; do not apply another 180-degree rotation in the sensor TF.

The G1 default is MID360s at 192.168.123.120, receiving on 192.168.123.166.
Set host_ip to the receiving computer's actual LiDAR-facing address. For a
different model supply a matching SDK configuration with livox_config:=PATH.
The SDK headers and shared library must both support MID360s.

Both the standalone msg_MID360_launch.py and Lightning's g1_online.launch.py
resolve the configuration from the installed livox_ros_driver2 package.
The standalone launch and robot_bringup sensor_driver.launch.py accept
livox_config:=PATH. Do not launch two drivers for the same sensor.

Outputs for the G1 launch:
- /livox/lidar: livox_ros_driver2/msg/CustomMsg, frame mid360_link
- /livox/imu: sensor_msgs/msg/Imu, frame mid360_imu
- The existing workspace converter provides /livox/points for RViz.

Build from the workspace after sourcing ROS:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --packages-select livox_ros_driver2 --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

URDF and Lightning core are not changed by this driver adaptation. If both
sensor frames represent Rx(pi)-corrected axes, the native IMU displacement
[0.011, 0.02329, -0.04412] must be expressed in those axes as
[0.011, -0.02329, 0.04412]. The G1 launch derives SLAM extrinsics from the URDF;
the owner must check that displacement before mapping.

Validate on hardware while stationary: messages arrive on both topics, IMU
acceleration Z becomes approximately +1 in the driver's existing g units,
and raw cloud orientation is correct in base_link. Compilation alone does
not validate hardware communications or physical extrinsics.
