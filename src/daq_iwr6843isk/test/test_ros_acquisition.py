"""Real ROS graph with simulated UARTs; never opens physical sensor devices."""

import json
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from daq_iwr6843isk.pcl_publisher import RadarPublisher, make_cloud, send_command
from daq_iwr6843isk.protocol import decode_packet
from test_acquisition import CFG, packet


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.buffer = bytearray()
        self.is_open = True
        self.lock = threading.Lock()
        self.is_cli = str(args[0]).endswith('cli')

    def inject(self, data):
        with self.lock:
            self.buffer.extend(data)

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.buffer)

    def read(self, size):
        with self.lock:
            result = bytes(self.buffer[:size])
            del self.buffer[:size]
        if not result:
            time.sleep(0.003)
        return result

    def write(self, data):
        if self.is_cli:
            self.inject(data + b'Done\nmmwDemo:/>')
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self.lock:
            self.buffer.clear()

    def close(self):
        self.is_open = False


def test_cloud_layout_and_empty_scan():
    nonempty = make_cloud(decode_packet(packet(1, [(1, 2, 3, -0.5)], [(123, 210)])), 1_234_567_890)
    assert nonempty.header.stamp.sec == 1
    assert nonempty.header.stamp.nanosec == 234_567_890
    np.testing.assert_allclose(np.frombuffer(nonempty.data, '<f4'), [1, 2, 3, 12.3, -0.5, 12.3, 21])
    empty = make_cloud(decode_packet(packet(2)), 2)
    assert empty.width == 0 and empty.row_step == 0 and bytes(empty.data) == b''
    missing = make_cloud(decode_packet(packet(3, [(1, 2, 3, 0)])), 3)
    assert np.isnan(np.frombuffer(missing.data, '<f4')[3])
    assert not missing.is_dense


def test_cli_rejection_is_not_configuration_success():
    class Reject(FakeSerial):
        def write(self, data):
            self.inject(b'Error: invalid argument\nmmwDemo:/>')
    with pytest.raises(RuntimeError, match='rejected'):
        send_command(Reject('cli'), 'aoaFovCfg invalid')


@pytest.mark.parametrize('duration,send_frames', [(0.0, True), (0.12, True), (0.0, False)])
def test_ros_driver_no_replay_empty_frames_and_recording(tmp_path, monkeypatch, duration, send_frames):
    ports = {}

    def factory(port, *args, **kwargs):
        ports[port] = FakeSerial(port, *args, **kwargs)
        return ports[port]

    monkeypatch.setattr('daq_iwr6843isk.pcl_publisher.serial.Serial', factory)
    cfg = tmp_path / 'radar.cfg'
    cfg.write_text(CFG)
    arguments = ['--ros-args']
    values = {'cfg_path': str(cfg), 'cli_port': '/tmp/simulated_cli',
              'data_port': '/tmp/simulated_data', 'record_trial': 'true',
              'plane_distance_m': '1.0', 'incidence_angle_deg': '30.0',
              'trial_id': 'simulated', 'output_root': str(tmp_path / 'output'),
              'duration_s': str(duration), 'scan_timeout_s': '3.0' if send_frames else '0.12'}
    for name, value in values.items():
        arguments.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=arguments)
    driver = observer = executor = None
    try:
        driver = RadarPublisher()
        observer = Node('radar_test_observer', use_global_arguments=False)
        clouds, infos = [], []
        cloud_sub = observer.create_subscription(PointCloud2, '/iwr6843_pcl', clouds.append, 100)
        info_sub = observer.create_subscription(String, '/iwr6843_scan_info', lambda m: infos.append(json.loads(m.data)), 100)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        executor.add_node(observer)
        deadline = time.monotonic() + 2
        while driver.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert driver.publisher.get_subscription_count() > 0
        if send_frames:
            ports['/tmp/simulated_data'].inject(
                packet(7, [(0, 2, 0, 0)], [(200, 300)]) + packet(8)
                + packet(10, [(0, 2, 0, 0)]) + packet(10, [(0, 2, 0, 0)]))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        if not send_frames:
            assert driver.finished and driver.stop_reason == 'scan_timeout'
            assert not clouds and not infos
            assert driver.received_frames == 0 and driver.empty_frames == 0
            driver.close()
            summary = json.loads((driver.recorder.path / 'summary.json').read_text())
            assert summary['received_unique_frames'] == summary['empty_frames'] == 0
            assert summary['stop_reason'] == 'scan_timeout'
            assert not summary['duration_completed']
            return
        assert [m.width for m in clouds] == [1, 0, 1]
        assert [m['frame_number'] for m in infos] == [7, 8, 10]
        assert infos[-1]['missing_frames_before'] == 1
        assert driver.sequence.duplicates == 1
        assert driver.received_frames == 3
        if duration:
            assert driver.finished and driver.stop_reason == 'duration_completed'
        else:
            assert not driver.finished
        path = driver.recorder.path
        driver.close()
        summary = json.loads((path / 'summary.json').read_text())
        assert summary['received_unique_frames'] == 3
        assert summary['empty_frames'] == 1
        assert summary['diagnostics']['missing_frame_numbers'] == 1
        assert summary['settings']['incidence_angle_deg'] == 30.0
        assert (path / 'uart.bin').stat().st_size > 0
    finally:
        if driver:
            driver.close()
        if executor:
            executor.shutdown()
        for node in (driver, observer):
            if node:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
