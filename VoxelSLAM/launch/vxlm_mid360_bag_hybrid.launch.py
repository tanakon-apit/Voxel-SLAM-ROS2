"""Replay a bag through VoxelSLAM with EMULATED IMU dropouts, in RViz.

Same as vxlm_mid360_bag.launch.py, but inserts scripts/imu_dropout.py between
the bag's raw IMU (/imu/data) and the SLAM node, which subscribes to the gapped
topic (/imu/data_gap, set in config/mid360_hybrid.yaml). This punches periodic
timed holes in the IMU so the LIO/LO fallback (enable_lo_fallback) is exercised
on a bag whose IMU is actually continuous.

    source /opt/ros/humble/setup.bash
    source ~/livox_ws/install/setup.bash
    source ~/dti_ws/install/setup.bash
    ros2 launch voxel_slam vxlm_mid360_bag_hybrid.launch.py bag:=/path/to/bag

Gap schedule (bag-time, keyed on IMU header stamps so it is playback-rate
independent): clean for `gap_warmup` s, then every `gap_period` s drop IMU for
`gap_len` s. Defaults: 30 s warmup, a 1.5 s blackout every 30 s. Set
`dropout:=false` to run the same config with the IMU untouched (A/B baseline).

Everything shuts down when the bag finishes. The `exit code -11` at teardown is
the pre-existing segfault noted in vxlm_mid360_bag.launch.py, not this launch.
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
    config = os.path.join(pkg, 'config', 'mid360_hybrid.yaml')
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

    # The IMU dropout relay: /imu/data -> /imu/data_gap. It ALWAYS runs (the SLAM
    # subscribes to /imu/data_gap), so the A/B baseline goes through the same
    # topic path. `dropout:=false` makes it a pure pass-through (gap_len=0 ->
    # never drops); `dropout:=true` punches the periodic holes.
    dropout_on = arg('dropout').lower() in ('true', '1', 'yes')
    imu_dropout_node = Node(
        package='voxel_slam',
        executable='imu_dropout.py',
        name='imu_dropout',
        output='screen',
        parameters=[{
            'input_topic': arg('imu_in'),
            'output_topic': arg('imu_out'),
            'warmup_sec': float(arg('gap_warmup')),
            'gap_period_sec': float(arg('gap_period')),
            'gap_len_sec': float(arg('gap_len')) if dropout_on else 0.0,
        }],
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
        imu_dropout_node,
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
        # --- IMU dropout emulation ---
        DeclareLaunchArgument('dropout', default_value='true',
                              description='Insert the IMU dropout relay. '
                                          'false = raw IMU (A/B baseline).'),
        DeclareLaunchArgument('imu_in', default_value='/imu/data',
                              description="Raw IMU topic from the bag."),
        DeclareLaunchArgument('imu_out', default_value='/imu/data_gap',
                              description="Gapped IMU topic the SLAM subscribes "
                                          "to (must match imu_topic in "
                                          "mid360_hybrid.yaml)."),
        DeclareLaunchArgument('gap_warmup', default_value='30.0',
                              description='Seconds of clean IMU before the first '
                                          'gap (let the initializer finish).'),
        DeclareLaunchArgument('gap_period', default_value='30.0',
                              description='Seconds between the start of each gap.'),
        DeclareLaunchArgument('gap_len', default_value='1.5',
                              description='Seconds each IMU blackout lasts.'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
