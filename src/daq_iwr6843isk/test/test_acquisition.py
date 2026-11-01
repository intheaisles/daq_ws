"""Protocol, measurement semantics, and saved-condition regression tests."""

import csv
import json
import struct

import numpy as np
import pytest

from daq_iwr6843isk.experiment import PlaneROI, TrialRecorder, trial_settings
from daq_iwr6843isk.protocol import (
    MAGIC, FrameSequence, PacketStream, config_commands, decode_packet,
)


CFG = 'sensorStop\nprofileCfg 0\nframeCfg 0 2 16 0 33.333 1 0\nsensorStart\n'
ROI = {'frame_id': 'iwr6843_frame', 'center_m': [0, 2, 0],
       'normal': [0, 1, 0], 'horizontal_axis': [0, 0, 1],
       'width_m': 2.0, 'height_m': 2.0, 'plane_tolerance_m': 0.1,
       'edge_exclusion_m': 0.1}


def packet(number, points=(), side=None, side_first=False):
    points = np.asarray(points, dtype='<f4').reshape(-1, 4)
    tlvs = []
    if len(points):
        tlvs.append(struct.pack('<II', 1, points.nbytes) + points.tobytes())
    if side is not None:
        values = np.asarray(side, dtype='<i2').reshape(-1, 2)
        tlvs.insert(0 if side_first else len(tlvs),
                    struct.pack('<II', 7, values.nbytes) + values.tobytes())
    payload = b''.join(tlvs)
    length = ((40 + len(payload) + 31) // 32) * 32
    header = struct.pack('<8s8I', MAGIC, 0x03050004, length, 0xa6843,
                         number, number * 1000, len(points), len(tlvs), 0)
    return header + payload + bytes(length - 40 - len(payload))


def test_fragmented_and_coalesced_packets_include_zero_frame():
    wire = b'noise' + packet(1, [(0, 2, 0, 0)], [(123, 210)]) + packet(2) + packet(3, [(1, 2, 0, 0)])
    stream = PacketStream()
    frames = []
    for index in range(0, len(wire), 7):
        frames.extend(stream.feed(wire[index:index + 7]))
    assert [f.frame_number for f in frames] == [1, 2, 3]
    assert [len(f.points) for f in frames] == [1, 0, 1]
    assert frames[0].points[0, 4] == pytest.approx(12.3)
    assert np.isnan(frames[2].points[0, 4])
    assert stream.feed(b'') == []
    assert stream.discarded_bytes == 5
    assert [f.frame_number for f in PacketStream().feed(wire[5:])] == [1, 2, 3]


def test_side_info_order_and_unknown_tlv():
    frame = decode_packet(packet(4, [(0, 2, 0, -0.5)], [(220, 300)], side_first=True))
    np.testing.assert_allclose(frame.points[0], [0, 2, 0, -0.5, 22, 30])
    unknown = struct.pack('<II', 99, 4) + b'abcd'
    wire = struct.pack('<8s8I', MAGIC, 0x03050004, 52, 0xa6843, 5, 5, 0, 1, 0) + unknown
    assert decode_packet(wire).points.shape == (0, 6)


def test_malformed_packet_resync_and_truncation():
    bad = bytearray(packet(1, [(0, 2, 0, 0)]))
    struct.pack_into('<I', bad, 44, 99999)
    stream = PacketStream()
    assert [f.frame_number for f in stream.feed(bytes(bad) + packet(3))] == [3]
    assert stream.malformed_packets >= 1
    stream = PacketStream()
    assert stream.feed(packet(4)[:30]) == []
    assert [f.frame_number for f in stream.feed(packet(4)[30:])] == [4]
    with pytest.raises(ValueError):
        decode_packet(packet(4)[:30])


def test_sequence_duplicates_losses_wrap_and_restart():
    sequence = FrameSequence()
    assert sequence.accept(0xfffffffe) == (True, 0, False)
    assert sequence.accept(0xffffffff) == (True, 0, False)
    assert sequence.accept(0) == (True, 0, False)
    assert sequence.accept(0) == (False, 0, False)
    assert sequence.accept(3) == (True, 2, False)
    assert sequence.accept(1) == (True, 0, True)
    assert (sequence.missing, sequence.duplicates, sequence.discontinuities) == (2, 1, 1)


def test_config_is_legacy_and_start_is_last():
    assert config_commands(CFG)[-1] == 'sensorStart'
    with pytest.raises(ValueError):
        config_commands(CFG + 'frameCfg 0\n')
    with pytest.raises(ValueError):
        config_commands('advFrameCfg 0\n' + CFG)


@pytest.mark.parametrize('distance,angle,duration', [(0, 0, 1), (-1, 0, 1),
                          (1, 90, 1), (1, -90, 1), (1, float('nan'), 1),
                          (float('inf'), 0, 1), (1, 0, -1)])
def test_invalid_trial_parameters(distance, angle, duration):
    with pytest.raises(ValueError):
        trial_settings(distance, angle, 'trial', duration)


def test_finite_plane_roi_excludes_edges_and_other_surfaces():
    roi = PlaneROI(ROI)
    points = np.array([[0, 2, 0], [0, 2, 0.95], [0, 3, 0], [1.2, 2, 0]])
    assert roi.mask(points).tolist() == [True, False, False, False]
    assert roi.mask(np.empty((0, 6))).tolist() == []
    with pytest.raises(ValueError):
        PlaneROI({**ROI, 'normal': [0, 0, 0]})


@pytest.mark.parametrize('distance,angle,trial', [(1, 30, 'left_1'), (2.5, -45, 'right_2')])
def test_saved_conditions_zero_frames_and_snr_missing(tmp_path, distance, angle, trial):
    settings = trial_settings(distance, angle, trial, 30)
    recorder = TrialRecorder(tmp_path, settings, CFG, 'radar.cfg', PlaneROI(ROI))
    recorder.configured([{'command': 'sensorStart', 'response': 'Done'}])
    recorder.record(decode_packet(packet(1, [(0, 2, 0, 0)], [(220, 300)])), 100)
    recorder.record(decode_packet(packet(2)), 200)
    # Other objects exist, but no point in the glass ROI. Missing SNR is not 0 dB.
    recorder.record(decode_packet(packet(4, [(0, 4, 0, 0)])), 400, gap=1)
    recorder.close('duration_completed', {'missing_frame_numbers': 1})
    metadata = json.loads((recorder.path / 'trial.json').read_text())
    summary = json.loads((recorder.path / 'summary.json').read_text())
    assert metadata['settings'] == summary['settings'] == settings
    assert settings['reference_range_derived_m'] == pytest.approx(distance / np.cos(np.deg2rad(angle)))
    assert summary['roi_support_rate'] == pytest.approx(1 / 3)
    assert summary['empty_frames'] == 1
    assert summary['mean_of_roi_scan_snr_medians_db'] == 22
    scans = list(csv.DictReader((recorder.path / 'scans.csv').open()))
    assert scans[1]['point_count'] == '0'
    assert scans[1]['roi_snr_median_db'] == ''
    assert scans[2]['roi_point_count'] == '0'
    points = list(csv.DictReader((recorder.path / 'points.csv').open()))
    assert len(points) == 2
    assert points[1]['snr_db'] == ''
    assert (recorder.path / 'radar.cfg').read_text() == CFG
    recorder.close('interrupted', {})  # Idempotent; original summary survives.
    assert json.loads((recorder.path / 'summary.json').read_text())['duration_completed']


def test_no_roi_does_not_report_glass_support(tmp_path):
    recorder = TrialRecorder(tmp_path, trial_settings(1, 0, 'no_roi', 0), CFG, 'radar.cfg')
    recorder.record(decode_packet(packet(1, [(0, 2, 0, 0)])), 10)
    recorder.close('interrupted', {})
    summary = json.loads((recorder.path / 'summary.json').read_text())
    assert summary['roi_support_rate'] is None
    assert summary['mean_of_scan_snr_medians_db'] is None
    assert not summary['duration_completed']


def test_incidence_and_boresight_angles_are_saved_separately():
    settings = trial_settings(1, 0, 'edge', 1, off_axis_deg='30.0')
    assert settings['incidence_angle_deg'] == 0
    assert settings['reference_off_axis_deg'] == 30
    assert trial_settings(1, 30, 'unknown', 1)['reference_off_axis_deg'] is None
    with pytest.raises(ValueError):
        trial_settings(1, 0, 'bad', 1, off_axis_deg='nan')
