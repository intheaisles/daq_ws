# 거리·입사각 Radar 예비시험 인계

2026-10-22 사용자 요청으로 오늘 시험을 마쳤다. 다음 재개 예정은 2026-10-23이다.
다음 단계는 **정면 3 m 예비시험**이며, 이 단계의 실행·완료는 아직 확인하지 않았다.

## 유지할 설정

- 거리 peak grouping OFF / 도플러 ON: `cfarCfg` 마지막 값은 각각 `0`, `1`.
- 다음 기록에는 `radar_fov_vertical15.cfg` 사용: raw azimuth ±15°, elevation ±30°.
  현재 장착에서 raw azimuth는 위·아래 방향에 해당한다.
- 같은 유리 영역·센서 높이·정면 방향을 유지하고 센서를 고정한다.
- 거리 입력은 레이더 원점에서 유리면까지의 실제 수직거리다.
- 원호 이동 / 수직거리 고정 각도 sweep 방식은 아직 결정하지 않았다.
- 장착 YAML은 `calibrated: false`인 추정값이다.

## 완료한 기록과 결과

- FoV 기본→좁게→기본: `fov_wide_01`, `fov_narrow_01`, `fov_wide_02`.
  각각 600 / 601 / 600프레임 정상 종료. 누락·중복·파싱 오류·queue drop 0.
  약 1.099 m의 사후 선택 근거리 점은 모든 frame에서 존재했고 SNR 중앙값은
  16.4→16.6→16.6 dB였다. 모든 공통 영역 점의 불변성을 입증한 것은 아니다.
- 같은 좁은 cfg의 정면 1 m→2 m→1 m:
  `fov_narrow_01`, `pilot_d2_a0_narrow_01`, `pilot_d1_a0_narrow_return_01`.
  각 601프레임 정상 종료, 누락·중복·파싱 오류·queue drop 0.
  비교 구간의 거리 중앙값은 1.099→2.076→1.099 m,
  SNR 중앙값은 16.6→37.5→16.1 dB였다. 각 구간 점은 매 frame에 존재했다.
  약 0.977 m 약한 인접 점의 지지 비율은 최초 좁은 1 m 95.3%에서 복귀 1 m
  22.3%로 달랐다. 대표 점 유지와 주변 점 구성의 재현성을 구분한다.
- 사용자는 2 m도 같은 유리 영역·높이, 정면 고정이라고 확인했다.
  실제 pose의 독립 계측은 없다. 동일 산란점·유리 재질·SNR 증가 원인은 미확정이다.
- 모든 위 기록에서 UART 재파싱과 CSV 점·SNR 값 일치를 확인했다.
- 분석 영역은 사후 진단용이며 공식 유리 ROI가 아니다. 공식 유리 검출률로 쓰지 않는다.

## 저장 위치

- 원본: `outputs/radar_trials/` 아래 각 trial_id로 끝나는 디렉터리.
- FoV 분석: `outputs/analysis/fov_pilot/`.
- 최종 거리 왕복 비교: `outputs/analysis/distance_pilot_d1_d2_d1/`.
- 각 분석 폴더에 `analyze.py`, `comparison.json`, `comparison.png`, `report.md`가 있다.
- trial 기록은 CSV/JSON/원본 UART이며 rosbag을 생성하는 launch는 아니다.

## 재개 시 다음 명령

먼저 센서를 정면 3 m에 배치하고 높이·조준 영역을 맞춘다.
다른 센서 launch가 실행 중이면 중복 실행하지 않는다.

```bash
cd ~/workspace/daq_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch daq_bringup radar_distance_angle_test.launch.py \
  radar_config:="$PWD/src/daq_bringup/config/iwr6843isk/radar_fov_vertical15.cfg" \
  plane_distance_m:=3.0 incidence_angle_deg:=0.0 \
  off_axis_deg:=0.0 trial_id:=pilot_d3_a0_narrow_01 \
  duration_s:=20.0 rviz:=true
```

실행 후 정상 수신·누락, 근거리 점 위치와 SNR을 확인하고 다음 거리·각도 조건을 정한다.
