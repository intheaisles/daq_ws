"""Trial metadata, optional finite plane ROI, and loss-aware CSV recording."""

import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
from zoneinfo import ZoneInfo

import numpy as np


def number_or_none(value):
    return float(value) if math.isfinite(float(value)) else None


def trial_settings(plane_distance_m, incidence_angle_deg, trial_id, duration_s, off_axis_deg=None):
    distance, angle, duration = map(float, (plane_distance_m, incidence_angle_deg, duration_s))
    if not all(math.isfinite(v) for v in (distance, angle, duration)):
        raise ValueError('Trial numbers must be finite')
    if distance <= 0 or not -90 < angle < 90 or duration < 0:
        raise ValueError('Require plane_distance_m > 0, -90 < incidence_angle_deg < 90, duration_s >= 0')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.+-]*', str(trial_id)):
        raise ValueError('trial_id must use letters, digits, _, ., + or -')
    off_axis = None if off_axis_deg is None or str(off_axis_deg).strip() == '' else float(off_axis_deg)
    if off_axis is not None and (not math.isfinite(off_axis) or abs(off_axis) > 180):
        raise ValueError('off_axis_deg must be blank or a finite angle between -180 and 180')
    return {
        'plane_distance_m': distance,
        'incidence_angle_deg': angle,
        'reference_off_axis_deg': off_axis,
        'reference_range_derived_m': distance / math.cos(math.radians(angle)),
        'reference_range_assumption': 'fixed reference point at radar height; angle is relative to its plane normal',
        'angle_convention': 'signed sweep angle; magnitude is reference-point incidence angle',
        'trial_id': str(trial_id), 'requested_duration_s': duration,
        'units': {'distance': 'm', 'angle': 'deg', 'snr': 'dB', 'doppler': 'm/s'},
    }


class PlaneROI:
    """A measured rectangle on a plane, expressed in the raw radar frame."""

    def __init__(self, config):
        if config.get('frame_id') != 'iwr6843_frame':
            raise ValueError('ROI frame_id must be iwr6843_frame')
        self.config = config
        self.center = self._vector(config['center_m'])
        normal = self._vector(config['normal'])
        horizontal = self._vector(config['horizontal_axis'])
        if np.linalg.norm(normal) < 1e-8 or np.linalg.norm(horizontal) < 1e-8:
            raise ValueError('ROI axes must be nonzero')
        self.normal = normal / np.linalg.norm(normal)
        horizontal = horizontal - self.normal * np.dot(horizontal, self.normal)
        if np.linalg.norm(horizontal) < 1e-8:
            raise ValueError('ROI horizontal_axis must not be parallel to normal')
        self.horizontal = horizontal / np.linalg.norm(horizontal)
        self.vertical = np.cross(self.normal, self.horizontal)
        width, height, self.tolerance, inset = (
            float(config[name]) for name in
            ('width_m', 'height_m', 'plane_tolerance_m', 'edge_exclusion_m'))
        if not all(math.isfinite(v) for v in (width, height, self.tolerance, inset)):
            raise ValueError('ROI dimensions must be finite')
        if self.tolerance <= 0 or inset < 0 or min(width, height) <= 2 * inset:
            raise ValueError('ROI requires positive tolerance and dimensions larger than twice the inset')
        self.half_width = width / 2 - inset
        self.half_height = height / 2 - inset

    @staticmethod
    def _vector(value):
        result = np.asarray(value, dtype=float)
        if result.shape != (3,) or not np.all(np.isfinite(result)):
            raise ValueError('ROI vectors must contain three finite numbers')
        return result

    def mask(self, points):
        delta = points[:, :3] - self.center
        return ((np.abs(delta @ self.normal) <= self.tolerance)
                & (np.abs(delta @ self.horizontal) <= self.half_width)
                & (np.abs(delta @ self.vertical) <= self.half_height))


def load_roi(path):
    if not path:
        return None
    return PlaneROI(json.loads(Path(path).expanduser().read_text(encoding='utf-8')))


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


class TrialRecorder:
    def __init__(self, output_root, settings, cfg_text, cfg_path, roi=None, runtime=None, mounting=None):
        now = datetime.now(ZoneInfo('Asia/Seoul'))
        root = Path(output_root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / (now.strftime('%Y%m%d_%H%M%S_%f') + '_' + settings['trial_id'])
        self.path.mkdir()  # Never overwrite an existing trial.
        self.roi = roi
        self.mounting = mounting
        self.settings = settings
        self.frames = self.empty_frames = self.supporting_frames = 0
        self.points = self.snr_frames = self.roi_snr_frames = 0
        self.snr_sum = self.roi_snr_sum = 0.0
        self.first_stamp = self.last_stamp = None
        self.closed = False
        self.cfg_hash = hashlib.sha256(cfg_text.encode()).hexdigest()
        (self.path / 'radar.cfg').write_text(cfg_text, encoding='utf-8')
        if mounting:
            (self.path / 'mounting.yaml').write_text(mounting.text, encoding='utf-8')
        sources = self.path / 'code'
        sources.mkdir()
        source_hashes = {}
        for source in Path(__file__).resolve().parent.glob('*.py'):
            shutil.copyfile(source, sources / source.name)
            source_hashes[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
        source_dir = Path(__file__).resolve().parent
        git = {}
        for name, command in [('commit', ['rev-parse', 'HEAD']),
                              ('status', ['status', '--short'])]:
            result = subprocess.run(['git', '-C', str(source_dir), *command],
                                    capture_output=True, text=True, check=False)
            git[name] = result.stdout.strip() if result.returncode == 0 else None
        self.metadata = {
            'schema_version': 2, 'started_at': now.isoformat(), 'settings': settings,
            'raw_point_frame': 'iwr6843_frame',
            'mounted_point_frame': None if mounting is None else mounting.parent_frame,
            'mounting': None if mounting is None else mounting.metadata(),
            'cfg_source_path': str(cfg_path), 'cfg_sha256': self.cfg_hash,
            'roi': None if roi is None else roi.config, 'runtime': runtime or {},
            'code_sha256': source_hashes, 'git': git,
            'timestamp_semantics': 'ROS clock at host UART receipt; not hardware acquisition time',
            'snr_semantics': 'detected-point side info; missing SNR is blank, not zero',
            'status': 'starting',
        }
        self._write_metadata()
        self.scan_file = (self.path / 'scans.csv').open('w', newline='', encoding='utf-8')
        self.point_file = (self.path / 'points.csv').open('w', newline='', encoding='utf-8')
        self.raw_file = (self.path / 'uart.bin').open('wb')
        self.scan_writer = csv.writer(self.scan_file)
        self.point_writer = csv.writer(self.point_file)
        self.scan_writer.writerow([
            'frame_number', 'host_stamp_ns', 'cpu_cycles', 'point_count',
            'roi_point_count', 'all_point_snr_median_db', 'roi_snr_median_db',
            'missing_frames_before', 'sequence_discontinuity', 'snr_point_count'])
        self.point_writer.writerow([
            'frame_number', 'host_stamp_ns', 'point_index', 'x_m', 'y_m', 'z_m',
            'doppler_mps', 'snr_db', 'noise_db', 'in_roi',
            'mounted_x_m', 'mounted_y_m', 'mounted_z_m'])

    def _write_metadata(self):
        write_json(self.path / 'trial.json', self.metadata)

    def configured(self, responses):
        write_json(self.path / 'cli_responses.json', responses)
        self.metadata['status'] = 'recording'
        self._write_metadata()

    def record(self, frame, stamp_ns, gap=0, discontinuity=False):
        points = frame.points
        mask = None if self.roi is None else self.roi.mask(points)
        roi_count = None if mask is None else int(np.count_nonzero(mask))
        snr = points[np.isfinite(points[:, 4]), 4]
        roi_snr = np.empty(0) if mask is None else points[mask & np.isfinite(points[:, 4]), 4]
        snr_median = float(np.median(snr)) if len(snr) else None
        roi_median = float(np.median(roi_snr)) if len(roi_snr) else None
        mounted = None if self.mounting is None else self.mounting.apply(points[:, :3])
        self.scan_writer.writerow([frame.frame_number, stamp_ns, frame.cpu_cycles, len(points),
                                   roi_count, snr_median, roi_median, gap, int(discontinuity), len(snr)])
        for index, point in enumerate(points):
            self.point_writer.writerow([frame.frame_number, stamp_ns, index,
                                       *[number_or_none(v) for v in point],
                                       None if mask is None else int(mask[index]),
                                       *([None, None, None] if mounted is None else mounted[index].tolist())])
        self.frames += 1
        self.points += len(points)
        self.empty_frames += int(len(points) == 0)
        self.supporting_frames += int(roi_count is not None and roi_count > 0)
        if snr_median is not None:
            self.snr_frames += 1
            self.snr_sum += snr_median
        if roi_median is not None:
            self.roi_snr_frames += 1
            self.roi_snr_sum += roi_median
        if self.first_stamp is None:
            self.first_stamp = stamp_ns
        self.last_stamp = stamp_ns
        return {'point_count': len(points), 'roi_point_count': roi_count,
                'snr_median_db': snr_median, 'roi_snr_median_db': roi_median}

    def flush(self):
        self.scan_file.flush()
        self.point_file.flush()
        self.raw_file.flush()

    def close(self, reason, diagnostics):
        if self.closed:
            return
        self.closed = True
        self.flush()
        for stream in (self.scan_file, self.point_file, self.raw_file):
            stream.close()
        summary = {
            'settings': self.settings, 'cfg_sha256': self.cfg_hash,
            'stop_reason': reason, 'duration_completed': reason == 'duration_completed',
            'received_unique_frames': self.frames, 'total_points': self.points,
            'empty_frames': self.empty_frames,
            'host_stamp_span_s': None if self.first_stamp is None else
                (self.last_stamp - self.first_stamp) / 1e9,
            'roi_supporting_frames': None if self.roi is None else self.supporting_frames,
            'roi_support_rate': None if self.roi is None or self.frames == 0 else
                self.supporting_frames / self.frames,
            'mean_of_scan_snr_medians_db': self.snr_sum / self.snr_frames if self.snr_frames else None,
            'mean_of_roi_scan_snr_medians_db': self.roi_snr_sum / self.roi_snr_frames
                if self.roi_snr_frames else None,
            'diagnostics': diagnostics,
            'interpretation': 'Received frames only; losses are not zero-point scans. No ROI means no glass detection metric.',
        }
        write_json(self.path / 'summary.json', summary)
        self.metadata['status'] = reason
        self._write_metadata()
