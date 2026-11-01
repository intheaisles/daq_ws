"""Explicit raw-radar to mounting-parent transform, shared by CSV and viewer."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class Mounting:
    parent_frame: str
    child_frame: str
    rotation: np.ndarray
    translation: np.ndarray
    calibrated: bool
    text: str
    source_path: str
    quaternion_xyzw: tuple

    @classmethod
    def load(cls, path):
        source = Path(path).expanduser().resolve()
        text = source.read_text(encoding='utf-8')
        config = yaml.safe_load(text)
        if not isinstance(config, dict) or config.get('child_frame') != 'iwr6843_frame':
            raise ValueError('Mounting child_frame must be iwr6843_frame')
        parent = str(config.get('parent_frame', '')).strip()
        if not parent or parent == config['child_frame']:
            raise ValueError('Mounting parent frame must be nonempty and different from child')
        translation = np.array([config['translation_m'][key] for key in ('x', 'y', 'z')], dtype=float)
        roll, pitch, yaw = [float(config['rotation_rpy_rad'][key]) for key in ('roll', 'pitch', 'yaw')]
        if not np.all(np.isfinite([*translation, roll, pitch, yaw])):
            raise ValueError('Mounting translation and angles must be finite')
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        hr, hp, hy = roll / 2, pitch / 2, yaw / 2
        qr, qp, qy = math.sin(hr), math.sin(hp), math.sin(hy)
        wr, wp, wy = math.cos(hr), math.cos(hp), math.cos(hy)
        quaternion = (qr * wp * wy - wr * qp * qy,
                      wr * qp * wy + qr * wp * qy,
                      wr * wp * qy - qr * qp * wy,
                      wr * wp * wy + qr * qp * qy)
        return cls(parent, 'iwr6843_frame', rotation, translation,
                   config.get('calibrated') is True, text, str(source), quaternion)

    def apply(self, xyz):
        return np.asarray(xyz, dtype=float) @ self.rotation.T + self.translation

    def metadata(self):
        return {'parent_frame': self.parent_frame, 'child_frame': self.child_frame,
                'calibrated': self.calibrated, 'source_path': self.source_path,
                'sha256': hashlib.sha256(self.text.encode()).hexdigest(),
                'quaternion_xyzw': list(self.quaternion_xyzw),
                'rotation_matrix': self.rotation.tolist(), 'translation_m': self.translation.tolist()}
