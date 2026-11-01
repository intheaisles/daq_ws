"""Mounted current-scan markers and numeric reports for the RViz table."""

import json
import math
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

from .mounting import Mounting


def read_points(message):
    """Read named float32 fields, respecting byte order, point and row strides."""
    count = int(message.width) * int(message.height)
    if not count:
        return np.empty((0, 6), dtype=float)
    expected = (message.height - 1) * message.row_step + message.width * message.point_step
    if len(message.data) < expected:
        raise ValueError('Truncated PointCloud2')
    fields = {field.name: field for field in message.fields}
    result = np.full((count, 6), np.nan)
    for column, name in enumerate(('x', 'y', 'z', 'doppler', 'snr', 'noise')):
        source = fields.get(name)
        if source is None and name == 'snr':
            source = fields.get('intensity')
        if source is None:
            if column < 3:
                raise ValueError(f'PointCloud2 lacks {name}')
            continue
        if source.datatype != PointField.FLOAT32 or source.count != 1 or source.offset + 4 > message.point_step:
            raise ValueError(f'Unsupported PointCloud2 field {source.name}')
        result[:, column] = np.ndarray(
            shape=(message.height, message.width),
            dtype='>f4' if message.is_bigendian else '<f4', buffer=message.data,
            offset=source.offset, strides=(message.row_step, message.point_step)).reshape(-1)
    return result


def make_markers(message, mounting, show_labels=True, text_height=0.10):
    if message.header.frame_id != mounting.child_frame:
        raise ValueError(f'Expected {mounting.child_frame}, received {message.header.frame_id}')
    points = read_points(message)
    mounted = mounting.apply(points[:, :3])
    lifetime = Duration(sec=0, nanosec=500_000_000)

    def marker(namespace, number, kind):
        item = Marker()
        item.header.frame_id = mounting.parent_frame
        item.header.stamp = message.header.stamp
        item.ns, item.id, item.type = namespace, number, kind
        item.pose.orientation.w = 1.0
        item.lifetime = lifetime
        return item

    clear = marker('radar', 0, Marker.SPHERE_LIST)
    clear.action = Marker.DELETEALL
    spheres = marker('radar_points', 0, Marker.SPHERE_LIST)
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.06
    spheres.color.a = 1.0
    output = [clear, spheres]
    for index, (point, raw) in enumerate(zip(mounted, points)):
        if not np.all(np.isfinite(point)):
            continue
        snr = float(raw[4])
        if math.isfinite(snr):
            fraction = float(np.clip((snr - 5) / 30, 0, 1))
            color = ColorRGBA(r=fraction, g=1 - abs(2 * fraction - 1), b=1 - fraction, a=1.0)
        else:
            color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
        spheres.points.append(Point(x=float(point[0]), y=float(point[1]), z=float(point[2])))
        spheres.colors.append(color)
        if show_labels:
            label = marker('radar_labels', index, Marker.TEXT_VIEW_FACING)
            label.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2] + 0.07))
            label.scale.z = text_height
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'#{index}'
            output.append(label)
    # Detailed values live in the docked RViz table, not over nearby points.
    return MarkerArray(markers=output)


def panel_report(message, mounting, state='live'):
    """JSON for the RViz dock; null represents a missing numeric value."""
    points = read_points(message) if message is not None and state == 'live' else np.empty((0, 6))
    mounted = mounting.apply(points[:, :3])
    rows = []
    for index, (xyz, raw) in enumerate(zip(mounted, points)):
        if not np.all(np.isfinite(xyz)):
            continue
        rows.append({'id': index, 'xyz_m': xyz.tolist(),
                     'range_m': float(np.linalg.norm(raw[:3])),
                     'snr_db': float(raw[4]) if math.isfinite(raw[4]) else None})
    return {'state': state, 'frame': mounting.parent_frame,
            'sensor_frame': mounting.child_frame, 'calibrated': mounting.calibrated,
            'sensor_xyz_m': mounting.translation.tolist(),
            'sensor_quaternion_xyzw': list(mounting.quaternion_xyzw),
            'stamp_ns': (message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
            if message is not None else None,
            'points': rows}


def sensor_markers(mounting):
    """Persistent origin separate from observations, including during dropouts."""
    origin = Marker()
    origin.header.frame_id = mounting.parent_frame
    origin.ns, origin.id, origin.type = 'radar_origin', 0, Marker.SPHERE
    origin.pose.position = Point(**dict(zip(('x', 'y', 'z'), map(float, mounting.translation))))
    origin.pose.orientation.w = 1.0
    origin.scale.x = origin.scale.y = origin.scale.z = 0.10
    origin.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=1.0)
    label = Marker()
    label.header.frame_id = mounting.parent_frame
    label.ns, label.id, label.type = 'radar_origin', 1, Marker.TEXT_VIEW_FACING
    label.pose.position = Point(x=float(mounting.translation[0]), y=float(mounting.translation[1]),
                                z=float(mounting.translation[2] + 0.25))
    label.pose.orientation.w = 1.0
    label.scale.z = 0.10
    label.color = origin.color
    label.text = 'RADAR'
    return MarkerArray(markers=[origin, label])


class RadarPointViewer(Node):
    def __init__(self):
        super().__init__('radar_point_viewer')
        self.declare_parameter('mounting_file', '')
        self.declare_parameter('show_labels', True)
        self.declare_parameter('text_height_m', 0.10)
        self.declare_parameter('publish_mount_tf', True)
        self.mounting = Mounting.load(self.get_parameter('mounting_file').value)
        self.text_height = float(self.get_parameter('text_height_m').value)
        if not math.isfinite(self.text_height) or self.text_height <= 0:
            raise ValueError('text_height_m must be positive and finite')
        if self.get_parameter('publish_mount_tf').value:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.mounting.parent_frame
            transform.child_frame_id = self.mounting.child_frame
            translation = transform.transform.translation
            translation.x, translation.y, translation.z = map(float, self.mounting.translation)
            quaternion = transform.transform.rotation
            quaternion.x, quaternion.y, quaternion.z, quaternion.w = self.mounting.quaternion_xyzw
            self.tf_broadcaster.sendTransform(transform)
        self.publisher = self.create_publisher(MarkerArray, '/daq/view/radar/markers', 10)
        self.report_publisher = self.create_publisher(String, '/daq/view/radar/report', 1)
        self.origin_publisher = self.create_publisher(
            MarkerArray, '/daq/view/radar/origin',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.origin_publisher.publish(sensor_markers(self.mounting))
        self.subscription = self.create_subscription(PointCloud2, '/iwr6843_pcl', self.received, 20)
        self.last_received = None
        self.stale_cleared = False
        self.timer = self.create_timer(0.1, self.check_stale)
        self.get_logger().info(f'Viewer frame={self.mounting.parent_frame}; XYZ table uses mounting transform; raw cloud unchanged.')

    def received(self, message):
        try:
            markers = make_markers(message, self.mounting,
                                   self.get_parameter('show_labels').value, self.text_height)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.publisher.publish(markers)
        self.report_publisher.publish(String(data=json.dumps(panel_report(message, self.mounting), allow_nan=False)))
        self.last_received = time.monotonic()
        self.stale_cleared = False

    def check_stale(self):
        if self.last_received is None:
            self.report_publisher.publish(String(data=json.dumps(panel_report(None, self.mounting, 'waiting'))))
            return
        if self.last_received is not None and not self.stale_cleared and time.monotonic() - self.last_received > 0.5:
            clear = Marker(action=Marker.DELETEALL)
            clear.header.frame_id = self.mounting.parent_frame
            self.publisher.publish(MarkerArray(markers=[clear]))
            self.report_publisher.publish(String(data=json.dumps(panel_report(None, self.mounting, 'stale'))))
            self.stale_cleared = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadarPointViewer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
