#!/usr/bin/env python3
"""Relay an IMU topic while punching periodic timed holes in it.

Used to emulate the IMU dropouts of the Hesai sequence on a bag whose IMU is
actually continuous (e.g. x30_mid360), so the VoxelSLAM LIO/LO fallback can be
exercised. LiDAR is left untouched; only the IMU stream gets gaps, which is
exactly the "LiDAR advancing while IMU stalls" condition sync_packages watches
for.

    /imu/data  --(this node)-->  /imu/data_gap  --(subscribed by voxelslam)

Blackout schedule is keyed on the IMU message header stamp relative to the
first message received, so it is reproducible regardless of playback rate:

    t0        = stamp of the first message
    elapsed   = stamp - t0
    if elapsed < warmup:            pass   (let the initializer get clean IMU)
    phase = (elapsed - warmup) % gap_period
    if phase < gap_len:             DROP   (blackout)
    else:                           pass

Parameters (ros2 params):
    input_topic   (str,   default /imu/data)
    output_topic  (str,   default /imu/data_gap)
    warmup_sec    (float, default 30.0)  seconds of clean IMU before the 1st gap
    gap_period_sec(float, default 30.0)  seconds between the start of each gap
    gap_len_sec   (float, default 1.5)   seconds each blackout lasts
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ImuDropout(Node):
    def __init__(self):
        super().__init__('imu_dropout')

        self.input_topic = self.declare_parameter('input_topic', '/imu/data').value
        self.output_topic = self.declare_parameter('output_topic', '/imu/data_gap').value
        self.warmup = float(self.declare_parameter('warmup_sec', 30.0).value)
        self.gap_period = float(self.declare_parameter('gap_period_sec', 30.0).value)
        self.gap_len = float(self.declare_parameter('gap_len_sec', 1.5).value)

        # Subscribe best-effort so we accept the bag's publisher whatever its
        # reliability; publish reliable so voxelslam accepts us either way.
        # Deep queues: IMU is ~200 Hz.
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=2000)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=2000)

        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)
        self.sub = self.create_subscription(Imu, self.input_topic, self._cb, sub_qos)

        self.t0 = None
        self.in_gap = False
        self.n_passed = 0
        self.n_dropped = 0

        self.get_logger().info(
            f"IMU dropout relay: {self.input_topic} -> {self.output_topic} | "
            f"warmup={self.warmup}s, every {self.gap_period}s drop {self.gap_len}s")

    def _cb(self, msg):
        t = _stamp_to_sec(msg.header.stamp)
        if self.t0 is None:
            self.t0 = t
        elapsed = t - self.t0

        drop = False
        if elapsed >= self.warmup and self.gap_period > 0.0:
            phase = (elapsed - self.warmup) % self.gap_period
            drop = phase < self.gap_len

        if drop != self.in_gap:
            self.in_gap = drop
            if drop:
                self.get_logger().warn(
                    f"IMU BLACKOUT start @ bag t={elapsed:.2f}s "
                    f"(will last {self.gap_len}s)")
            else:
                self.get_logger().info(
                    f"IMU restored @ bag t={elapsed:.2f}s "
                    f"(passed={self.n_passed}, dropped={self.n_dropped})")

        if drop:
            self.n_dropped += 1
            return
        self.n_passed += 1
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuDropout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
