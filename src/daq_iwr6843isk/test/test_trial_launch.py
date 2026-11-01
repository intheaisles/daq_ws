"""Run the installed launch against pseudo terminals, never real hardware."""

import csv
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time
import tty

from test_acquisition import packet


def test_finite_trial_launch_with_pseudo_uart(tmp_path):
    cli_master, cli_slave = os.openpty()
    data_master, data_slave = os.openpty()
    tty.setraw(cli_slave)
    tty.setraw(data_slave)
    stop = threading.Event()
    sensor_running = threading.Event()
    commands = []
    errors = []

    def emulate():
        buffer = bytearray()
        frame = 0
        last = 0
        try:
            while not stop.is_set():
                readable, _, _ = select.select([cli_master], [], [], 0.005)
                if readable:
                    buffer.extend(os.read(cli_master, 4096))
                    while b'\n' in buffer:
                        command, _, rest = buffer.partition(b'\n')
                        buffer[:] = rest
                        command = command.decode().strip()
                        commands.append(command)
                        os.write(cli_master, (command + '\nDone\nmmwDemo:/>').encode())
                        if command == 'sensorStart':
                            sensor_running.set()
                        elif command == 'sensorStop':
                            sensor_running.clear()
                if sensor_running.is_set() and time.monotonic() - last > 1 / 30:
                    frame += 1
                    wire = packet(frame) if frame % 2 == 0 else packet(frame, [(0, 2, 0, 0)], [(200, 300)])
                    os.write(data_master, wire)
                    last = time.monotonic()
        except Exception as error:
            errors.append(str(error))

    worker = threading.Thread(target=emulate, daemon=True)
    worker.start()
    process = None
    try:
        environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
        process = subprocess.Popen([
            'ros2', 'launch', 'daq_bringup', 'radar_distance_angle_test.launch.py',
            'plane_distance_m:=1.0', 'incidence_angle_deg:=30.0', 'off_axis_deg:=0.0',
            'trial_id:=pty_trial', 'duration_s:=0.4', 'rviz:=false',
            f'cli_port:={os.ttyname(cli_slave)}', f'data_port:={os.ttyname(data_slave)}',
            f'output_root:={tmp_path / "trials"}',
        ], env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        output, _ = process.communicate(timeout=20)
        assert process.returncode == 0, output
        assert '(duration_completed)' in output, output
        assert not errors
        directories = list((tmp_path / 'trials').iterdir())
        assert len(directories) == 1
        path = directories[0]
        metadata = json.loads((path / 'trial.json').read_text())
        summary = json.loads((path / 'summary.json').read_text())
        assert metadata['settings']['plane_distance_m'] == 1
        assert metadata['settings']['incidence_angle_deg'] == 30
        assert metadata['settings']['reference_off_axis_deg'] == 0
        assert metadata['settings']['trial_id'] == 'pty_trial'
        assert summary['duration_completed']
        assert summary['received_unique_frames'] >= 8
        assert summary['empty_frames'] >= 3
        assert summary['roi_support_rate'] is None
        assert summary['diagnostics']['missing_frame_numbers'] == 0
        rows = list(csv.DictReader((path / 'scans.csv').open()))
        assert len(rows) == summary['received_unique_frames']
        assert len({row['frame_number'] for row in rows}) == len(rows)
        assert 'sensorStart' in commands and 'sensorStop' in commands
        assert (path / 'cli_responses.json').is_file()
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        stop.set()
        worker.join(timeout=2)
        for descriptor in (cli_master, cli_slave, data_master, data_slave):
            os.close(descriptor)
