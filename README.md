# Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System (ROS 2 Port)

This repository is a **ROS 2 (Humble)** port of the original ROS 1 [Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM) by HKU-MARS. The `voxel_slam` package builds with `ament_cmake`/`colcon` and runs as a single `rclcpp` node.

## 1. Introduction

**Voxel-SLAM** is a complete, accurate, and versatile LiDAR-inertial SLAM system that fully utilizes short-term, mid-term, long-term, and multi-map data associations. It includes five modules: initialization, odometry, local mapping, loop closure, and global mapping. The initialization can provide accurate states and a local map in a static or dynamic initial state. The odometry estimates current states and detects potential system divergence. The local mapping refines the states and local map within the sliding window by a LiDAR-inertial BA. The loop closure can detect loops across multiple sessions. The global mapping refines the global map with an efficient hierarchical global BA.

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
</div>

### 1.1 Related video

The video of **Voxel-SLAM** is available on [YouTube](https://youtu.be/Cg9W01aIUzE).

### 1.2 Related works

The related paper is available on [**arXiv**](https://arxiv.org/abs/2410.08935).

### 1.3 Competitions

Voxel-SLAM served as a subsystem in the [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html) (**2nd** place, LiDAR single-session) and the [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI) (**1st** place, LiDAR-inertial track).

## 2. Prerequisites

- Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [PCL](https://pointclouds.org/) and [Eigen](https://eigen.tuxfamily.org/) (the versions shipped with ROS 2 Humble work)
- [GTSAM](https://github.com/borglab/gtsam) ≥ 4.1 (the port is built with C++17)
- `traversability_msgs` (from [traversability_mapping](https://github.com/suchetanrs/traversability_mapping)) — message definitions for the optional traversability key-frame outputs (see §6). A build dependency even if you don't use that feature.
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (optional) — provides the `livox_ros_driver2/msg/CustomMsg` type, needed only for Livox sensors (`lidar_type: 0`). If it is not found at build time, the package builds without Livox support and the PointCloud2 LiDARs (Velodyne/Ouster/Hesai/…) still work; requesting `lidar_type: 0` then exits with an error at startup.

### 2.1 Installing the dependencies

**ROS 2 Humble** — follow the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then install the ROS packages this node uses (PCL and Eigen come in as their dependencies):

```bash
sudo apt install ros-humble-desktop            # or ros-humble-ros-base + ros-humble-rviz2
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions \
                 ros-humble-tf2-ros python3-colcon-common-extensions
```

**GTSAM** — either of these works:

```bash
# Option A: GTSAM 4.2 from the ROS repository (simplest, no extra PPA)
sudo apt install ros-humble-gtsam

# Option B: GTSAM 4.1 from the Borglab PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev
```

(Building [GTSAM](https://github.com/borglab/gtsam) 4.1/4.2 from source also works.)

> **Troubleshooting:** if `find_package(GTSAM)` fails with
> `The imported target "CppUnitLite" references the file "/usr/lib/x86_64-linux-gnu/libCppUnitLite.a" but this file does not exist`,
> you have the `libgtsam-dev` package from the plain Ubuntu archive (`4.1.1-1ubuntuXX`), which has a packaging bug: its CMake
> config exports the `CppUnitLite` target but the package doesn't ship the library. Either install GTSAM from one of the two
> sources above instead, or satisfy the check with an empty stub archive (nothing actually links against it — it is only
> GTSAM's internal test helper):
>
> ```bash
> sudo ar rc /usr/lib/x86_64-linux-gnu/libCppUnitLite.a
> ```
>
> If the build then fails with `No rule to make target '/usr/lib/x86_64-linux-gnu/libmetis.so'`, the same Ubuntu GTSAM
> package links against system METIS but only the runtime library (`libmetis5`) is installed; the unversioned `.so`
> comes from the dev package:
>
> ```bash
> sudo apt install libmetis-dev
> ```

**traversability_msgs** — clone the repository into your workspace `src/` and build the message package (the rest of that repository is not required):

```bash
cd <your_ros2_ws>/src
git clone https://github.com/suchetanrs/traversability_mapping
cd ..
colcon build --packages-select traversability_msgs
```

**livox_ros_driver2** (optional, for Livox sensors) — the driver needs the [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) library first:

```bash
# 1. Livox-SDK2 (system-wide install)
git clone https://github.com/Livox-SDK/Livox-SDK2
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j
sudo make install

# 2. the ROS 2 driver, in its own workspace (its build script requires it)
mkdir -p <livox_driver_ws>/src && cd <livox_driver_ws>/src
git clone https://github.com/Livox-SDK/livox_ros_driver2
cd livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

Then source `<livox_driver_ws>/install/setup.bash` before building or running `voxel_slam` (alternatively, clone the driver into the same workspace `src/` so it builds alongside):

```bash
source /opt/ros/humble/setup.bash
source <livox_driver_ws>/install/setup.bash   # provides livox_ros_driver2
```

## 3. Build

```bash
cd <your_ros2_ws>
source /opt/ros/humble/setup.bash
source <livox_driver_ws>/install/setup.bash   # optional, only for Livox support (see §2)
colcon build --packages-select voxel_slam
source install/setup.bash
```

## 4. Run Voxel-SLAM

Each supported sensor has a launch file and a matching config YAML in `VoxelSLAM/config/`:

| Launch file | Config | Sensor |
|---|---|---|
| `vxlm_mid360.launch.py` | `mid360.yaml` | Livox Mid-360 |
| `vxlm_mid360_bag.launch.py` | `mid360.yaml` | Livox Mid-360 (bag replay, see §4.2) |
| `vxlm_avia.launch.py` | `avia.yaml` | Livox Avia (handheld) |
| `vxlm_avia_fly.launch.py` | `avia_fly.yaml` | Livox Avia (aerial, MARS dataset) |
| `vxlm_hesai.launch.py` | `hesai.yaml` | Hesai (HILTI 2023, multi-session) |
| `vxlm_ouster.launch.py` | `ouster.yaml` | Ouster |
| `vxlm_velodyne.launch.py` | `velodyne.yaml` | Velodyne |

All launch files accept `rviz:=false` to disable RViz2.

### 4.1 Live / generic usage

```bash
ros2 launch voxel_slam vxlm_mid360.launch.py
# or run the node directly:
ros2 run voxel_slam voxelslam --ros-args \
  --params-file $(ros2 pkg prefix voxel_slam)/share/voxel_slam/config/mid360.yaml
```

To replay data manually, play a ROS 2 bag in another terminal:

```bash
ros2 bag play <your_bag> 
```

When a run is finished, trigger the final global mapping (global bundle adjustment) and map save with:

```bash
ros2 param set /voxelslam finish true
```

This replaces the ROS 1 `rosparam set finish true`.

> Original ROS 1 datasets (Avia elevator/relocalization, HILTI 2023, MARS, Mid-360 jungle) are linked in the [upstream repository](https://github.com/hku-mars/Voxel-SLAM). ROS 1 bags must be converted (e.g. with `rosbags`) before they can be replayed under ROS 2.

### 4.2 Bag replay (Mid-360)

`vxlm_mid360_bag.launch.py` starts the node and RViz2, plays a bag after the node is subscribed, and shuts everything down when the bag ends:

```bash
ros2 launch voxel_slam vxlm_mid360_bag.launch.py bag:=/path/to/bag_dir
```

Arguments:

- `bag` — bag directory to replay
- `rate` (default `1.0`) — playback rate; above ~1.0 the odometry thread may drop scans
- `start_offset` (default `0.0`) — seconds to skip into the bag
- `delay` (default `4.0`) — seconds to wait before playing so the node is subscribed first (a bag started too early loses the IMU samples the initializer needs)
- `rviz` (default `true`)

Note: at teardown the node may report `process has died ... exit code -11`. This segfault is pre-existing upstream behaviour (worker threads outlive the context while publishers are destroyed on SIGINT); it happens after the run is over and does not affect results.

### 4.3 Multi-session mapping

Multi-session works as in the original system, configured through the `General` parameters (now in ROS 2 YAML format, see §5):

```yaml
General:
  save_path: "/path/to/save/offline/maps/"
  previous_map: "session_a: 0.50, session_b: 0.45"  # sessions to load, with their loop thresholds; "" for none
  bagname: "session_c"                              # name of the current session
  is_save_map: 1                                    # enable to save the map
```

Run each session in turn; sessions listed in `previous_map` are loaded from `save_path` and the loop closure can relocalize across them. After each session, run `ros2 param set /voxelslam finish true` for the final global BA before starting the next.

## 5. Configuration

Config files are ROS 2 parameter YAMLs under a `/**: ros__parameters:` root. The ROS 1 `A/B` parameter keys map to ROS 2's nested `A.B` (e.g. `General/lid_topic` → `General: lid_topic:`). All floating-point parameters must be written with decimals — ROS 2 is strict about int-vs-double types.

Key `General` parameters:

- `lid_topic`, `imu_topic` — input topics
- `lidar_type` — `0` Livox (CustomMsg), `1` Velodyne, `2` Ouster, `3` Hesai (PointCloud2)
- `extrinsic_tran`, `extrinsic_rota` — LiDAR-IMU extrinsics
- `world_frame` (default `camera_init`), `body_frame` (default `aft_mapped`), and optional `odom_frame` — TF frames (leading slashes removed for ROS 2). If `odom_frame` is set, TF is published as `world → odom → body` with the loop correction applied to `world → odom`.
- `odom_topic` — if set, publishes `nav_msgs/Odometry` on this topic (plus `/Odometry_Corrected`)
- `save_path`, `previous_map`, `bagname`, `is_save_map` — multi-session map saving/loading (§4.3)

`Odometry` extras added by this port: `imu_cv_fallback` bridges IMU dropouts with a constant-velocity motion model instead of silently dropping scans — see [VoxelSLAM/doc/imu_cv_fallback.md](VoxelSLAM/doc/imu_cv_fallback.md) for the design and state diagram.

## 6. Topics

**Subscribed**

- IMU: `sensor_msgs/Imu` on `imu_topic`
- LiDAR: `livox_ros_driver2/msg/CustomMsg` (Livox) or `sensor_msgs/PointCloud2` (others) on `lid_topic`

**Published**

- `/map_scan`, `/map_cmap`, `/map_pmap`, `/map_init`, `/map_path`, `/map_true`, `/map_test` — `sensor_msgs/PointCloud2` map/scan/path visualizations
- `/slam_kf_path`, `/slam_path_corrected`, `/slam_path_continuous` — `nav_msgs/Path`
- `/slam_degenerate` — `std_msgs/Header` divergence/reset flags (`degenerate`, `reset`)
- `nav_msgs/Odometry` on `odom_topic` and `/Odometry_Corrected` (only if `odom_topic` is set)
- TF: `world_frame → body_frame` (or `world → odom → body`, see §5)
- Optional (`pub_trav_keyframes: true`): `traversability_msgs` key-frame additions/updates for downstream traversability mapping, gated by `trav_kf_dist`

## 7. Changes from the ROS 1 version

- `roscpp` → `rclcpp`; a single node `voxelslam`, spun by a `SingleThreadedExecutor` in its own thread (replaces `ros::spinOnce()` in the odometry loop).
- Messages: `livox_ros_driver/CustomMsg` → `livox_ros_driver2/msg/CustomMsg`.
- TF: `tf::TransformBroadcaster` → `tf2_ros::TransformBroadcaster`.
- Parameters: `nh.param()` → a declare-on-demand `get_param()` helper; config YAMLs converted to ROS 2 format.
- Time: `stamp.toSec()`/`fromSec()` → helpers in `src/tools.hpp`; wall timing uses monotonic `std::chrono`.
- Build: catkin → `ament_cmake`, C++17.
- `config/ouster.yaml`: upstream used section names `feature:`/`EKF:` that the node never reads; corrected to `General:`/`Odometry:`.

See [VoxelSLAM/README_ROS2.md](VoxelSLAM/README_ROS2.md) for the full porting notes.

## 8. VoxelSLAMPointCloud2 (not ported)

`VoxelSLAMPointCloud2` is the original ROS 1 RViz display plugin (a PointCloud2 display that clears the map automatically on an empty cloud). It is kept in the tree for reference but marked with `COLCON_IGNORE` and is **not built**. RViz2's built-in PointCloud2 display covers the same need; the provided `VoxelSLAM/rviz_cfg/voxelslam.rviz` uses it.

## License

See [LICENSE](LICENSE). Original work by [HKU-MARS Lab](https://github.com/hku-mars/Voxel-SLAM).
