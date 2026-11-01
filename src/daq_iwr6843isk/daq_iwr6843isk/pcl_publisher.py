# Copyright 2016 Open Source Robotics Foundation, Inc.
# Copyright 2026 DAQ workspace contributors
# Licensed under the Apache License, Version 2.0.
"""Publish each complete TI hardware frame once, including zero-point frames."""

import hashlib
import json
from pathlib import Path
import queue
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import serial
from std_msgs.msg import String

from .experiment import TrialRecorder, load_roi, trial_settings
from .protocol import FrameSequence, PacketStream, config_commands, config_summary
from .mounting import Mounting
from .paths import default_output_root


# Keep intensity as the SNR alias for existing RViz layouts; also expose named fields.
CLOUD_FIELDS = [('x', 0), ('y', 4), ('z', 8), ('intensity', 12),
                ('doppler', 16), ('snr', 20), ('noise', 24)]


def make_cloud(frame, stamp_ns):
    points = frame.points
    packed = np.column_stack((points[:, :3], points[:, 4], points[:, 3], points[:, 4:])).astype('<f4')
    message = PointCloud2()
    message.header.frame_id = 'iwr6843_frame'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
    message.height = 1
    message.width = len(points)
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in CLOUD_FIELDS]
    message.is_bigendian = False
    message.point_step = 28
    message.row_step = 28 * len(points)
    message.is_dense = bool(np.all(np.isfinite(packed)))
    message.data = packed.tobytes()
    return message


def send_command(port, command, timeout_s=8.0):
    """Wait for the OOB CLI response; never assume writing means applied."""
    port.write((command + '\n').encode('ascii'))
    port.flush()
    response = bytearray()
    deadline = time.monotonic() + timeout_s
    last_data = time.monotonic()
    while time.monotonic() < deadline:
        data = port.read(max(1, port.in_waiting))
        if data:
            response.extend(data)
            last_data = time.monotonic()
        text = response.decode('utf-8', errors='replace')
        lowered = text.lower()
        if any(token in lowered for token in ('error', 'exception', 'not recognized', 'invalid usage')):
            raise RuntimeError(f'Radar rejected {command!r}: {text.strip()}')
        done = 'Done' in text or (command == 'sensorStop' and 'already stopped' in lowered)
        if done and (text.rstrip().endswith(':/>') or time.monotonic() - last_data >= 0.15):
            return text
    raise RuntimeError(f'Radar CLI timeout for {command!r}: {response.decode(errors="replace")}')


class RadarPublisher(Node):
    def __init__(self):
        super().__init__('iwr6843_pcl_pub')
        defaults = {
            'cfg_path': '', 'cli_port': '/dev/ttyUSB0', 'data_port': '/dev/ttyUSB1',
            'record_trial': False, 'output_root': default_output_root(), 'mounting_file': '',
            'plane_distance_m': 0.0, 'incidence_angle_deg': 0.0, 'trial_id': 'trial1',
            'duration_s': 0.0, 'roi_file': '', 'scan_timeout_s': 3.0, 'off_axis_deg': '',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.parameters = {key: self.get_parameter(key).value for key in defaults}
        self.finished = False
        self.closed = False
        self.stop_reason = 'interrupted'
        self.cli = self.data = self.recorder = None
        self.reader = None
        self.stop_event = threading.Event()
        self.pending = queue.Queue(maxsize=2048)
        self.parser = PacketStream()
        self.sequence = FrameSequence()
        self.reader_error = None
        self.queue_dropped = 0
        self.raw_bytes = 0
        self.received_frames = 0
        self.reported_frames = 0
        self.empty_frames = 0
        self.last_scan_details = {}
        self.startup_error = None
        self.first_receive = self.last_receive = None
        self.started = self.last_report = time.monotonic()
        self.cli_responses = []
        self.publisher = self.create_publisher(PointCloud2, '/iwr6843_pcl', 100)
        self.info_publisher = self.create_publisher(String, '/iwr6843_scan_info', 100)
        try:
            self._start()
        except Exception as error:
            self.startup_error = str(error)
            self.close('startup_error')
            self.destroy_node()
            raise
        self.timer = self.create_timer(0.005, self.tick)

    def _start(self):
        p = self.parameters
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Live acquisition requires use_sim_time=false')
        if not np.isfinite(p['scan_timeout_s']) or p['scan_timeout_s'] <= 0:
            raise ValueError('scan_timeout_s must be finite and positive')
        if not np.isfinite(p['duration_s']) or p['duration_s'] < 0:
            raise ValueError('duration_s must be finite and nonnegative')
        cfg_path = Path(p['cfg_path']).expanduser().resolve()
        cfg_text = cfg_path.read_text(encoding='utf-8')
        commands = config_commands(cfg_text)
        self.cfg_summary = config_summary(cfg_text)
        self.cfg_hash = hashlib.sha256(cfg_text.encode()).hexdigest()
        self.mounting = Mounting.load(p['mounting_file']) if p['mounting_file'] else None
        if Path(p['cli_port']).resolve() == Path(p['data_port']).resolve():
            raise ValueError('CLI and data ports must be different')
        if p['record_trial']:
            settings = trial_settings(p['plane_distance_m'], p['incidence_angle_deg'],
                                      p['trial_id'], p['duration_s'], p['off_axis_deg'])
            self.recorder = TrialRecorder(p['output_root'], settings, cfg_text, cfg_path,
                                          load_roi(p['roi_file']), runtime={
                                              'parameters': p, 'argv': sys.argv,
                                              'radar_config': self.cfg_summary}, mounting=self.mounting)
            self.get_logger().info(f'Trial output: {self.recorder.path}')
            if self.recorder.roi is None:
                self.get_logger().info('No ROI: recording ALL radar points; glass support rate is unavailable.')
        self.cli = serial.Serial(p['cli_port'], 115200, timeout=0.05, write_timeout=2)
        self.data = serial.Serial(p['data_port'], 921600, timeout=0.05)
        self.cli.reset_input_buffer()
        self.data.reset_input_buffer()
        # Stop first even if the supplied config omitted sensorStop.
        if commands[0] != 'sensorStop':
            commands.insert(0, 'sensorStop')
        for command in commands:
            response = send_command(self.cli, command)
            self.cli_responses.append({'command': command, 'response': response})
        if self.recorder:
            self.recorder.configured(self.cli_responses)
        self.started = self.last_report = time.monotonic()
        self.reader = threading.Thread(target=self._read, name='radar-uart', daemon=True)
        self.reader.start()
        self.get_logger().info('Radar started; one cloud per hardware frame, including empty frames.')

    def _read(self):
        try:
            while not self.stop_event.is_set():
                data = self.data.read(min(max(1, self.data.in_waiting), 65536))
                if not data:
                    continue
                stamp = self.get_clock().now().nanoseconds
                received = time.monotonic()
                self.raw_bytes += len(data)
                if self.recorder:
                    self.recorder.raw_file.write(data)
                for frame in self.parser.feed(data):
                    try:
                        self.pending.put_nowait((frame, stamp, received))
                    except queue.Full:
                        self.queue_dropped += 1
        except Exception as error:
            if not self.stop_event.is_set():
                self.reader_error = str(error)

    def tick(self):
        if self.finished:
            return
        duration = self.parameters['duration_s']
        for _ in range(256):
            try:
                frame, stamp, received = self.pending.get_nowait()
            except queue.Empty:
                break
            if self.first_receive is not None and duration > 0 and received - self.first_receive > duration:
                self.stop_reason = 'duration_completed'
                self.finished = True
                break
            accepted, gap, discontinuity = self.sequence.accept(frame.frame_number)
            if not accepted:
                continue
            if self.first_receive is None:
                self.first_receive = received
            self.last_receive = received
            self.received_frames += 1
            self.empty_frames += int(len(frame.points) == 0)
            details = self.recorder.record(frame, stamp, gap, discontinuity) if self.recorder else {}
            self.last_scan_details = details
            info = {'frame_number': frame.frame_number, 'host_stamp_ns': stamp,
                    'cpu_cycles': frame.cpu_cycles, 'subframe_number': frame.subframe_number,
                    'sdk_version_header': frame.version, 'platform_header': frame.platform,
                    'point_count': len(frame.points), 'missing_frames_before': gap,
                    'sequence_discontinuity': discontinuity, 'cfg_sha256': self.cfg_hash,
                    **details}
            if self.recorder:
                info['trial'] = self.recorder.settings
                info['trial_directory'] = str(self.recorder.path)
            self.publisher.publish(make_cloud(frame, stamp))
            self.info_publisher.publish(String(data=json.dumps(info, allow_nan=False)))
        now = time.monotonic()
        if self.reader_error:
            self.get_logger().error(f'UART reader failed: {self.reader_error}')
            self.stop_reason, self.finished = 'serial_error', True
        elif not self.finished and now - (self.last_receive or self.started) > self.parameters['scan_timeout_s']:
            self.get_logger().error('No fresh hardware frames; NOT treating timeout as zero detections.')
            self.stop_reason, self.finished = 'scan_timeout', True
        elif not self.finished and self.first_receive is not None and duration > 0 and now - self.first_receive >= duration:
            # Drain queued observations before ending a finite trial.
            if self.pending.empty():
                self.stop_reason, self.finished = 'duration_completed', True
        if now - self.last_report >= 2:
            hz = (self.received_frames - self.reported_frames) / (now - self.last_report)
            self.reported_frames = self.received_frames
            self.last_report = now
            text = (f'Radar={hz:.1f} Hz, hardware frames={self.received_frames}, empty={self.empty_frames}, '
                    f'missing={self.sequence.missing}, duplicates={self.sequence.duplicates}, '
                    f'parse_errors={self.parser.malformed_packets}, queue_dropped={self.queue_dropped}')
            if self.recorder:
                recorder = self.recorder
                text += f', ROI support={recorder.supporting_frames}/{recorder.frames}' if recorder.roi else ', ROI=unset (all-point data)'
                snr_key = 'roi_snr_median_db' if recorder.roi else 'snr_median_db'
                snr = self.last_scan_details.get(snr_key)
                text += f', last scan SNR={snr:.1f} dB' if snr is not None else ', last scan SNR=unavailable'
                recorder.flush()
            self.get_logger().info(text)

    def diagnostics(self):
        return {'missing_frame_numbers': self.sequence.missing,
                'duplicate_frames_dropped': self.sequence.duplicates,
                'sequence_discontinuities': self.sequence.discontinuities,
                'malformed_packets': self.parser.malformed_packets,
                'discarded_uart_bytes': self.parser.discarded_bytes,
                'incomplete_uart_bytes_at_stop': len(self.parser.buffer),
                'queue_dropped_frames': self.queue_dropped,
                'unprocessed_frames_at_stop': self.pending.qsize(),
                'raw_uart_bytes': self.raw_bytes, 'reader_error': self.reader_error,
                'startup_error': self.startup_error}

    def close(self, reason=None):
        if self.closed:
            return
        self.closed = True
        self.stop_event.set()
        if self.reader:
            self.reader.join(timeout=2)
        try:
            if self.cli and self.cli.is_open:
                self.cli.write(b'sensorStop\n')
                self.cli.flush()
        except (OSError, serial.SerialException):
            pass
        finally:
            for port in (self.data, self.cli):
                if port and port.is_open:
                    port.close()
        if self.recorder:
            # Preserve successful command responses even if startup failed later.
            from .experiment import write_json
            write_json(self.recorder.path / 'cli_responses.json', self.cli_responses)
            self.recorder.close(reason or self.stop_reason, self.diagnostics())
            self.get_logger().info(f'Trial saved: {self.recorder.path} ({reason or self.stop_reason})')


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = RadarPublisher()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.stop_reason not in ('duration_completed', 'interrupted'):
            code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'Radar acquisition failed: {error}', file=sys.stderr)
        if node:
            node.stop_reason = 'processing_error'
        code = 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == '__main__':
    main()
