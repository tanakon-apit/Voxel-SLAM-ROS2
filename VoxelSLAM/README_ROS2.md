# Voxel-SLAM — ROS 2 (Humble) port

This is a ROS 2 port of the original ROS 1 `voxel_slam` package.

## Build
```bash
cd ~/dti_ws
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash   # provides livox_ros_driver2 (CustomMsg)
colcon build --packages-select voxel_slam
source install/setup.bash
```

## Run
```bash
ros2 launch voxel_slam vxlm_mid360.launch.py           # rviz:=false to disable RViz2
# or directly:
ros2 run voxel_slam voxelslam --ros-args \
  --params-file src/Voxel-SLAM/VoxelSLAM/config/mid360.yaml
```
Set `finish` to stop and trigger the final global BA / map save:
```bash
ros2 param set /voxelslam finish true
```

## What changed vs the ROS 1 version
- `roscpp` -> `rclcpp`; single node `voxelslam`, spun by a `SingleThreadedExecutor`
  in its own thread (replaces `ros::spinOnce()` in the odometry loop).
- Messages: `sensor_msgs/Imu`, `sensor_msgs/PointCloud2`,
  `livox_ros_driver/CustomMsg` -> `livox_ros_driver2/msg/CustomMsg`.
- TF: `tf::TransformBroadcaster` -> `tf2_ros::TransformBroadcaster`
  (`camera_init` -> `aft_mapped`, leading slashes removed for ROS 2).
- Parameters: `nh.param()` -> a `get_param()` helper that declares-on-demand and
  maps `A/B` keys to ROS 2's `A.B`. Config YAMLs are ROS 2 format under `/**`.
  All floating-point params/arrays are written with decimals (ROS 2 is strict
  about int-vs-double override types).
- Time: `header.stamp.toSec()` -> `toSec(stamp)`; `stamp.fromSec()` ->
  `fromSec(stamp, s)`; timing `ros::Time::now().toSec()` -> `get_time_sec()`
  (monotonic std::chrono). Helpers live in `src/tools.hpp`.
- Build: catkin -> ament_cmake, C++17 (required by GTSAM 4.2 / ROS 2 Humble).
- `config/ouster.yaml`: upstream used section names `feature:`/`EKF:` that the
  node never reads; corrected to `General:`/`Odometry:`.

## Not ported
`VoxelSLAMPointCloud2` is a ROS 1 RViz **display plugin** (custom PointCloud2
display). It is marked `COLCON_IGNORE`. RViz2's built-in PointCloud2 display
covers the same need; the provided `rviz_cfg/voxelslam.rviz` uses it.
