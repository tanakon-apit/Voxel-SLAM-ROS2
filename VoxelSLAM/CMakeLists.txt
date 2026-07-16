cmake_minimum_required(VERSION 3.8)
project(voxel_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# ROS 2 Humble / GTSAM 4.2 build with C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3)

# ---- ament / ROS 2 dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# Livox support is optional: without livox_ros_driver2 the node still builds
# and runs every PointCloud2 lidar_type, it just cannot subscribe to CustomMsg.
find_package(livox_ros_driver2 QUIET)

# ---- third-party ----
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(traversability_msgs REQUIRED)

set(ROS_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  traversability_msgs
)
if(livox_ros_driver2_FOUND)
  list(APPEND ROS_DEPS livox_ros_driver2)
  add_compile_definitions(VOXELSLAM_WITH_LIVOX)
  message(STATUS "voxel_slam: building WITH Livox CustomMsg support")
else()
  message(STATUS "voxel_slam: livox_ros_driver2 not found, building WITHOUT Livox CustomMsg support (lidar_type 0 disabled)")
endif()

add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp)

target_include_directories(voxelslam PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

ament_target_dependencies(voxelslam ${ROS_DEPS})

target_link_libraries(voxelslam
  ${PCL_LIBRARIES}
  gtsam
)

install(TARGETS voxelslam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})

# IMU dropout relay (emulate IMU loss on a continuous-IMU bag). Runnable as
# `ros2 run voxel_slam imu_dropout.py` and used by vxlm_mid360_bag_hybrid.launch.py.
install(PROGRAMS scripts/imu_dropout.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
