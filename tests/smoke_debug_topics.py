"""Run with ROS and robot_interfaces sourced; uses a PTY, never robot hardware."""
import functools
import math
import operator
import os
import pty
import signal
import struct
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robot_interfaces.msg import ImuEuler
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def packet(kind, payload):
    data = bytes([0xAA, 0x55, kind, len(payload)]) + payload
    return data + bytes([functools.reduce(operator.xor, data)])


def main():
    master, slave = pty.openpty()
    process = subprocess.Popen(
        ["/ws/install/lib/stm32_driver/stm32_driver_node", "--ros-args",
         "-p", "port:=" + os.ttyname(slave)],
        stdout=subprocess.DEVNULL,
    )
    rclpy.init()
    node = rclpy.create_node("debug_topic_smoke_test")
    debug, imu, odom, dynamic, static = [], [], [], [], []
    subscriptions = [
        node.create_subscription(ImuEuler, "/debug/imu", debug.append, 10),
        node.create_subscription(Imu, "/imu", imu.append, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom/solo", odom.append, 10),
        node.create_subscription(TFMessage, "/tf", dynamic.append, 10),
        node.create_subscription(
            TFMessage, "/tf_static", static.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            assert process.poll() is None, "driver exited"
            # Raw roll includes the configured half-turn mounting correction.
            data = [math.pi + 0.3, -0.2, 0.4, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            os.write(master, packet(1, struct.pack("<I15f", 0, *data)))
            os.write(master, packet(2, struct.pack("<I4f2I", 0, 0, 0, 60, 60, 0, 0)))
            rclpy.spin_once(node, timeout_sec=0.02)
            if debug and imu and len(odom) > 10 and static:
                break
        assert debug and imu and len(odom) > 10 and static, "missing output topics"
        for field, expected in [("roll", 0.3), ("pitch", -0.2), ("yaw", 0.4)]:
            assert math.isclose(getattr(debug[-1], field + "_rad"), expected, abs_tol=1e-5)
            assert math.isclose(
                getattr(debug[-1], field + "_deg"), math.degrees(expected), abs_tol=1e-3)
        assert debug[-1].header.frame_id == "imu_link"
        assert math.isclose(imu[-1].linear_acceleration.z, 9.80665, abs_tol=1e-5)
        assert odom[-1].header.frame_id == "enc_odom"
        assert odom[-1].child_frame_id == "base_link"
        assert odom[-1].pose.pose.position.x > odom[0].pose.pose.position.x
        assert abs(odom[-1].pose.pose.position.y) < 1e-9
        assert math.isclose(odom[-1].twist.twist.linear.x, 2 * math.pi * 0.0346)
        assert not dynamic, "driver must not publish dynamic TF"
        assert {(t.header.frame_id, t.child_frame_id)
                for msg in static for t in msg.transforms} == {("chassis", "imu_link")}
        assert "/enc/odom" not in dict(node.get_topic_names_and_types())
        print("PASS: corrected Euler radians/degrees, SI acceleration, encoder-only odometry, no diagnostic TF")
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
