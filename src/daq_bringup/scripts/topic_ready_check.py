#!/usr/bin/env python3
"""Wait for one message on every required DAQ topic before recording."""

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


MESSAGE_TYPES = {
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/CompressedImage": CompressedImage,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "tf2_msgs/msg/TFMessage": TFMessage,
}


class TopicReadyCheck(Node):
    def __init__(self):
        super().__init__("daq_topic_ready_check")
        self.declare_parameter("required_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("timeout_sec", 20.0)

        specifications = list(
            self.get_parameter("required_topics").get_parameter_value().string_array_value
        )
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        if not specifications:
            raise ValueError("required_topics must not be empty")
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")

        self.pending = set()
        self._daq_subscriptions = []
        self.started_at = time.monotonic()
        self.last_report_at = 0.0
        self.finished = False
        self.succeeded = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        for specification in specifications:
            try:
                topic, type_name = specification.split("=", maxsplit=1)
                message_type = MESSAGE_TYPES[type_name]
            except (KeyError, ValueError) as error:
                raise ValueError(
                    f"Invalid required topic specification: {specification}"
                ) from error

            self.pending.add(topic)
            qos = static_tf_qos if topic == "/tf_static" else sensor_qos
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda _message, received_topic=topic: self._received(received_topic),
                qos,
            )
            self._daq_subscriptions.append(subscription)

        self.timer = self.create_timer(0.25, self._check)
        self.get_logger().info(
            f"Waiting for the first message on {len(self.pending)} DAQ topics"
        )

    def _received(self, topic: str) -> None:
        if topic in self.pending:
            self.pending.remove(topic)
            self.get_logger().info(f"Ready: {topic}")

    def _check(self) -> None:
        now = time.monotonic()
        if not self.pending:
            self.succeeded = True
            self.finished = True
            self.get_logger().info("All required DAQ topics are publishing")
            self.timer.cancel()
            return

        if now - self.started_at >= self.timeout_sec:
            missing = "\n  ".join(sorted(self.pending))
            self.get_logger().error(
                f"DAQ preflight timed out. Missing messages:\n  {missing}"
            )
            self.finished = True
            self.timer.cancel()
            return

        if now - self.last_report_at >= 2.0:
            self.last_report_at = now
            self.get_logger().info(
                f"Still waiting for {len(self.pending)} topic(s)"
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = TopicReadyCheck()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
        return 0 if node.succeeded else 1
    except KeyboardInterrupt:
        return 130
    except Exception as error:  # launch must receive a non-zero result.
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"DAQ preflight error: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
