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
find_package(livox_ros_driver2 REQUIRED)

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
  livox_ros_driver2
  traversability_msgs
)

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

ament_package()
