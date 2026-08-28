#!/usr/bin/env python3
"""Print two-second DAQ sensor receipt rates in the bag launch terminal."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2


MESSAGE_TYPES = {
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/CompressedImage": CompressedImage,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}


class TopicRateMonitor(Node):
    def __init__(self):
        super().__init__("daq_topic_rate_monitor")
        self.declare_parameter("monitored_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("report_period_sec", 2.0)
        specifications = list(
            self.get_parameter("monitored_topics").get_parameter_value().string_array_value
        )
        self.report_period = float(self.get_parameter("report_period_sec").value)
        if not specifications:
            raise ValueError("monitored_topics must not be empty")
        if self.report_period <= 0.0:
            raise ValueError("report_period_sec must be positive")

        self.entries = []
        self.counts = {}
        self.seen = set()
        self._daq_rate_subscriptions = []
        self.window_started = time.monotonic()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for specification in specifications:
            parts = specification.split("=", maxsplit=3)
            if len(parts) != 4:
                raise ValueError(f"Invalid monitor specification: {specification}")
            topic, type_name, label, expected_text = parts
            try:
                message_type = MESSAGE_TYPES[type_name]
                expected_hz = float(expected_text)
            except (KeyError, ValueError) as error:
                raise ValueError(
                    f"Invalid monitor specification: {specification}"
                ) from error
            if expected_hz <= 0.0:
                raise ValueError(f"Expected rate must be positive: {specification}")

            self.entries.append((topic, label, expected_hz))
            self.counts[topic] = 0
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda _message, received_topic=topic: self._received(received_topic),
                qos,
            )
            self._daq_rate_subscriptions.append(subscription)

        self.timer = self.create_timer(self.report_period, self._report)
        self.get_logger().info(
            f"Reporting DAQ sensor rates every {self.report_period:.1f} seconds"
        )

    def _received(self, topic: str) -> None:
        self.counts[topic] += 1
        self.seen.add(topic)

    def _report(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self.window_started, 1.0e-6)
        fields = []
        low_labels = []

        for topic, label, expected_hz in self.entries:
            count = self.counts[topic]
            actual_hz = count / elapsed
            if topic not in self.seen:
                status = "MISSING"
                low_labels.append(label)
            elif actual_hz < 0.9 * expected_hz:
                status = "LOW"
                low_labels.append(label)
            else:
                status = "OK"
            fields.append(
                f"{label}={actual_hz:.1f}/{expected_hz:.0f}Hz[{status}]"
            )
            self.counts[topic] = 0

        message = " | ".join(fields)
        if low_labels:
            self.get_logger().warning(f"DAQ Hz: {message}")
        else:
            self.get_logger().info(f"DAQ Hz: {message}")
        self.window_started = now


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = TopicRateMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"DAQ rate monitor error: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
