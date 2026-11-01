"""Mounted geometry, visible XYZ/SNR, stale clearing and saved provenance."""

import csv
import json
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from daq_iwr6843isk.experiment import TrialRecorder, trial_settings
from daq_iwr6843isk.mounting import Mounting
from daq_iwr6843isk.paths import default_output_root, workspace_root
from daq_iwr6843isk.pcl_publisher import make_cloud
from daq_iwr6843isk.protocol import decode_packet
from daq_iwr6843isk.viewer import RadarPointViewer, make_markers, read_points, panel_report, sensor_markers
from std_msgs.msg import String
from test_acquisition import CFG, packet


MOUNT = Path(__file__).resolve().parents[2] / 'daq_bringup/config/extrinsics/d455_iwr6843isk.yaml'


def test_current_mount_axes_and_labels():
    mounting = Mounting.load(MOUNT)
    axes = mounting.apply(np.eye(3)) - mounting.translation
    np.testing.assert_allclose(axes, [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)
    message = make_cloud(decode_packet(packet(1, [(0, 2, 0, 0)], [(220, 300)])), 12)
    markers = make_markers(message, mounting).markers
    assert markers[0].action == Marker.DELETEALL
    spheres = next(item for item in markers if item.ns == 'radar_points')
    np.testing.assert_allclose([spheres.points[0].x, spheres.points[0].y, spheres.points[0].z],
                               [1.99, -0.045, -0.07])
    label = next(item for item in markers if item.ns == 'radar_labels')
    assert label.header.frame_id == 'camera_link'
    assert label.text == '#0'
    report = panel_report(message, mounting)
    np.testing.assert_allclose(report['points'][0]['xyz_m'], [1.99, -0.045, -0.07])
    assert report['points'][0]['snr_db'] == 22.0
    assert report['points'][0]['range_m'] == 2.0
    np.testing.assert_allclose(report['sensor_xyz_m'], mounting.translation)
    origin = sensor_markers(mounting).markers[0]
    np.testing.assert_allclose([origin.pose.position.x, origin.pose.position.y, origin.pose.position.z], mounting.translation)
    assert origin.lifetime.sec == origin.lifetime.nanosec == 0
    assert label.lifetime.nanosec > 0
    assert not mounting.calibrated
    np.testing.assert_allclose(mounting.quaternion_xyzw, [-0.5, -0.5, -0.5, 0.5])


def test_empty_and_missing_snr_are_not_old_points():
    mounting = Mounting.load(MOUNT)
    empty = make_markers(make_cloud(decode_packet(packet(2)), 20), mounting).markers
    assert empty[0].action == Marker.DELETEALL
    assert not next(item for item in empty if item.ns == 'radar_points').points
    assert not any(item.ns == 'radar_labels' for item in empty)
    assert panel_report(make_cloud(decode_packet(packet(2)), 20), mounting)['points'] == []
    assert panel_report(None, mounting, 'stale')['points'] == []
    for side, expected in [(None, None), ([(0, 0)], 0.0)]:
        message = make_cloud(decode_packet(packet(3, [(0, 2, 0, 0)], side)), 30)
        report = panel_report(message, mounting)
        assert report['points'][0]['snr_db'] == expected
        json.dumps(report, allow_nan=False)
    message.header.frame_id = 'unexpected'
    with pytest.raises(ValueError):
        make_markers(message, mounting)


def test_point_reader_handles_padding_and_big_endian():
    message = make_cloud(decode_packet(packet(1, [(1, 2, 3, 0)], [(120, 200)])), 1)
    message.width, message.height, message.row_step = 1, 2, 32
    values = np.array([1, 2, 3, 12, 0, 12, 20], dtype='>f4').tobytes()
    message.is_bigendian = True
    message.data = values + bytes(4) + values + bytes(4)
    points = read_points(message)
    assert points.shape == (2, 6)
    np.testing.assert_allclose(points[:, :3], [[1, 2, 3], [1, 2, 3]])
    assert points[0, 4] == 12


def test_mounted_csv_and_config_snapshot(tmp_path):
    mounting = Mounting.load(MOUNT)
    recorder = TrialRecorder(tmp_path, trial_settings(1, 0, 'mount', 1), CFG, 'radar.cfg', mounting=mounting)
    recorder.record(decode_packet(packet(1, [(0, 2, 0, 0)], [(200, 300)])), 1)
    recorder.close('duration_completed', {})
    metadata = json.loads((recorder.path / 'trial.json').read_text())
    assert metadata['raw_point_frame'] == 'iwr6843_frame'
    assert metadata['mounted_point_frame'] == 'camera_link'
    assert not metadata['mounting']['calibrated']
    assert (recorder.path / 'mounting.yaml').read_text() == MOUNT.read_text()
    row = next(csv.DictReader((recorder.path / 'points.csv').open()))
    assert float(row['y_m']) == 2
    assert float(row['mounted_x_m']) == pytest.approx(1.99)
    assert Path(default_output_root()).is_relative_to(workspace_root())
    assert Path(default_output_root('daq_bags')).is_relative_to(workspace_root())


def test_viewer_ros_current_empty_and_timeout():
    rclpy.init(args=['--ros-args', '-p', f'mounting_file:={MOUNT}'])
    viewer = source = executor = None
    try:
        viewer = RadarPointViewer()
        source = Node('viewer_test_source', use_global_arguments=False)
        publisher = source.create_publisher(PointCloud2, '/iwr6843_pcl', 10)
        received = []
        reports = []
        report_subscription = source.create_subscription(String, '/daq/view/radar/report', reports.append, 10)
        subscription = source.create_subscription(MarkerArray, '/daq/view/radar/markers', received.append, 10)
        transforms = []
        transform_subscription = source.create_subscription(
            TFMessage, '/tf_static', transforms.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        executor = SingleThreadedExecutor()
        executor.add_node(source)
        executor.add_node(viewer)
        deadline = time.monotonic() + 2
        while (publisher.get_subscription_count() == 0 or viewer.publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert publisher.get_subscription_count() > 0
        deadline = time.monotonic() + 1
        while not transforms and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert transforms
        transform = transforms[0].transforms[0]
        assert transform.header.frame_id == 'camera_link'
        assert transform.child_frame_id == 'iwr6843_frame'
        for wire in (packet(1, [(0, 2, 0, 0)], [(220, 300)]), packet(2)):
            before = len(received)
            publisher.publish(make_cloud(decode_packet(wire), source.get_clock().now().nanoseconds))
            deadline = time.monotonic() + 1
            while len(received) == before and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert len(received) > before
        assert any(item.ns == 'radar_labels' for item in received[0].markers)
        assert not any(item.ns == 'radar_labels' for item in received[1].markers)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(received[-1].markers) == 1
        assert received[-1].markers[0].action == Marker.DELETEALL
        assert reports
        assert json.loads(reports[-1].data)['state'] == 'stale'
        assert json.loads(reports[-1].data)['points'] == []
    finally:
        if executor:
            executor.shutdown()
        for node in (viewer, source):
            if node:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
