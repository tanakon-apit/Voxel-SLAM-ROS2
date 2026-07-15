"""Replay a bag through VoxelSLAM, in RViz.

    ros2 launch voxel_slam vxlm_mid360_bag.launch.py bag:=/path/to/bag

Everything shuts down when the bag finishes, so this leaves nothing running.

Expect `process has died ... exit code -11` from the voxelslam node at teardown.
That segfault is pre-existing: the node crashes destroying its publishers on
SIGINT (worker threads outlive the context -- see the warning on pub_pl_func in
voxelslam.hpp). It happens after the run is over and costs nothing, but it is
not caused by this launch file.

The livox_ros_driver2 typesupport library lives in a separate workspace; source
it before launching or the node dies with a missing-shared-library error:

    source /opt/ros/humble/setup.bash
    source ~/livox_ws/install/setup.bash
    source ~/dti_ws/install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_a, **_kw):
    pkg = get_package_share_directory('voxel_slam')
    config = os.path.join(pkg, 'config', 'mid360.yaml')
    rviz_config = os.path.join(pkg, 'rviz_cfg', 'voxelslam.rviz')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    voxelslam_node = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        # Line-buffer stdout: the node reports progress with printf, which is
        # block-buffered when it is not a tty.
        prefix='stdbuf -oL -eL',
        parameters=[config],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # `ros2 bag play` rejects `--start-offset 0.0` outright ("not in the valid
    # range (> 0.0)"), so the flag has to be omitted rather than passed as zero.
    cmd = ['ros2', 'bag', 'play', arg('bag'), '--rate', arg('rate')]
    if float(arg('start_offset')) > 0.0:
        cmd += ['--start-offset', arg('start_offset')]
    bag_play = ExecuteProcess(cmd=cmd, output='screen')

    return [
        voxelslam_node,
        rviz_node,
        TimerAction(period=float(arg('delay')), actions=[bag_play]),
        # Tear the whole stack down when the bag runs out, rather than leaving
        # the node and RViz alive waiting for scans that never come.
        RegisterEventHandler(OnProcessExit(target_action=bag_play,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bag', default_value='/home/tanakon/x30_mid360',
                              description='Bag directory to replay.'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Bag playback rate. Above ~1.0 the '
                                          'odometry thread may drop scans.'),
        DeclareLaunchArgument('start_offset', default_value='0.0',
                              description='Seconds to skip into the bag.'),
        DeclareLaunchArgument('delay', default_value='4.0',
                              description='Seconds to wait before playing, so '
                                          'the node is subscribed first. A bag '
                                          'that starts too early loses the IMU '
                                          'samples the initializer needs.'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
