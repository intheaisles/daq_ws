# 거리·입사각 Radar 시험 실행

작성: 2026-10-22. 원호 이동과 수직 이격거리 고정 중 어떤 배치를 사용할지는
예비 테스트 데이터를 본 뒤 사용자가 결정한다. 현재는 미정이며 코드는 이동
경로를 강제하지 않는다. `plane_distance_m`는 배치와 무관하게 각 측정 위치에서의
실제 수직거리다. 같은 높이의 고정 유리 기준점에 대한 각도를 사용한다면
원호 반경 r과 각도 θ에서 이 입력값은 r*cos(θ)가 된다.
현재 cfg의 각도 범위·CFAR·장착축은 이 코드 추가로 변경하지 않았다.

## 장착 방향을 반영한 viewer

`rviz:=true`로 시험을 실행하면 장착 YAML을 적용한 `camera_link` 좌표로 점을
표시한다. X=전방, Y=왼쪽, Z=위쪽이다. 점 옆에는 `#번호`만 표시하고,
RViz의 `Radar live values` 패널에 번호별 XYZ [m], 센서 원점에서의 거리 R [m],
SNR [dB]를 표로 표시한다. SNR이 없으면 N/A이며 점 색상은 SNR에 따라 바뀐다.
텍스트 번호는 현재 scan 안의 점 번호이며, 물리 표적의 추적 ID는 아니다.

기본 장착 파일은 `config/extrinsics/d455_iwr6843isk.yaml`이다.
`calibrated: false` 상태를 그대로 보존하고 화면에도 mounting: estimate로 표시한다.
원본 `/iwr6843_pcl`의 좌표나 frame_id를 바꾸지 않고 viewer용 MarkerArray만
`/daq/view/radar/markers`로 발행한다. 단독 viewer는 장착 TF를 발행하고,
combined sensor view에서는 기존 TF를 사용해 중복 발행하지 않는다.
combined viewer에도 같은 표시를 사용한다. 표의 데이터는 `/daq/view/radar/report`
JSON String이다. XYZ는 YAML 변환을 한 번 적용한 부모 좌표이며, RViz는 마커의
frame_id를 기준으로 표시한다. 기본 fixed frame은 같은 부모 좌표라 추가 변환은 없다.

노란 구와 RADAR 라벨은 센서 원점이고, RGB 축은 각각 해당 프레임의 X/Y/Z다.
기본 센서 원점은 camera_link에서 (-0.010, -0.045, -0.070) m이며 패널에도 표시한다.
이는 장착 상대 위치이고, map/odom의 이동 중 현재 위치를 추정하는 기능은 없다.
좌표 원점과 축은 관측이 중단돼도 유지한다.

정상 0점 scan이면 이전 점·라벨을 지우고 points=0을 표시한다. 새 scan이 0.5초
이상 없으면 점과 표의 관측값을 지우고 NO FRESH SCAN을 표시한다.
과거 점을 새 관측처럼 보여주지 않는다. `show_point_labels:=false`는 점 옆 번호만
숨기며, 수치 표는 유지한다. 새 패널은 빌드 및 source 후 RViz를 다시 열면 적용된다.

이미 Radar가 실행 중일 때 viewer만 열려면:

```bash
ros2 launch daq_bringup radar_view.launch.py
```

이 명령은 센서와 기록을 시작하지 않는다. 시험 launch의 rviz=true와 별도로
viewer를 중복 실행하지 않는다.

## 빌드

```bash
cd ~/workspace/daq_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

## 시험 실행

이 launch가 Radar를 직접 시작한다. 같은 장비의 기존 radar launch가 실행 중이면
먼저 해당 터미널에서 종료하고 포트를 해제한다. D455를 시작할 필요는 없다.

```bash
ros2 launch daq_bringup radar_distance_angle_test.launch.py \
  plane_distance_m:=1.0 incidence_angle_deg:=30.0 \
  off_axis_deg:=0.0 trial_id:=d1_a30_repeat1 duration_s:=30.0 \
  rviz:=true
```

- `plane_distance_m`: 사용자 실측 수직 이격거리 [m], 양수. 기준점까지 직선거리와 다르다.
- `incidence_angle_deg`: 고정 기준점에 대한 입사각 [deg]. 좌우 접근 방향은
  사용자가 일관된 부호로 구분한다. 범위는 -90° 초과, 90° 미만이다.
- `off_axis_deg`: 기준점 방향과 Radar 정면축 사이의 각도 [deg], 선택 항목.
  기준점을 정면으로 조준하면 0. FoV 가장자리 실험에서는 해당 값을 입력한다.
  생략하면 미상(null)으로 저장한다. 좌우 부호 규약은 시행 간 동일하게 사용한다.
- `trial_id`: 반복·조건 식별자. 영문/숫자/`_ . + -` 사용.
- `duration_s`: 첫 정상 hardware frame 수신부터 기록할 시간. 기본 30초는 실행
  편의상 초기값이며 통계적으로 충분한 시간이라는 뜻은 아니다. 0이면 Ctrl-C까지 기록한다.
- `output_root`: 기본 `~/workspace/daq_ws/outputs/radar_trials`. 각 실행은 시각과 trial_id로 고유 디렉터리를 만든다.
- `roi_file`: 유리 내부의 유한 평면 영역 JSON. 기본은 미지정이며 **전체 Radar 점만 기록**한다.
- `scan_timeout_s`: 정상 frame이 새로 들어오지 않으면 중단하는 시간. 기본 3초.
  정상적인 0점 frame도 정상 수신이므로 timeout을 일으키지 않는다.

화면의 전체 점 수나 전체 점 SNR은 유리 검출 통계가 아니다. ROI가 없으면
`roi_support_rate`는 null이며 유리 검출률을 출력하지 않는다.

포트나 cfg를 바꿀 때:

```bash
ros2 launch daq_bringup radar_distance_angle_test.launch.py \
  plane_distance_m:=1.0 incidence_angle_deg:=-30.0 \
  trial_id:=d1_am30_repeat1 duration_s:=30.0 \
  cli_port:=/dev/ttyUSB0 data_port:=/dev/ttyUSB1 \
  radar_config:=/absolute/path/to/limited_fov.cfg
```

원래 cfg와 제한 cfg 비교 때는 같은 위치·조준·공통 분석 영역을 사용한다.
각 실행의 cfg 사본·SHA-256·CLI 응답을 결과 폴더에 저장한다. 설정 명령이
거절되거나 응답하지 않으면 기록을 정상 시작한 것으로 처리하지 않는다.
지원 대상은 **TI SDK 3.x legacy frameCfg Out-of-Box packet 형식**이다.

## 결과

```text
~/workspace/daq_ws/outputs/radar_trials/YYYYMMDD_HHMMSS_microseconds_<trial_id>/
  trial.json          # 입력 조건·단위·기하 가정·실행 인자·코드/설정 식별·종료 상태
  radar.cfg           # 이 실행이 실제로 CLI에 보낸 cfg의 원문 사본
  mounting.yaml        # 표시·변환에 사용한 장착 설정 사본
  cli_responses.json  # cfg 명령별 장비 응답
  scans.csv           # 0점 frame을 포함한 scan별 집계·frame 번호·누락·SNR
  points.csv          # 개별 점 x/y/z, Doppler, SNR, noise, ROI 포함 여부
  uart.bin            # 수신한 원본 UART 바이트. 재파싱·오류 분석용
  summary.json        # 조건별 요약과 수신/파싱/queue 상태
  code/               # 실행한 driver·parser·recorder Python 소스 사본
```

`reference_range_derived_m`은 입력 수직거리/cos(기준점 입사각)로 계산한다.
고정 기준점이 Radar와 같은 높이에 있다는 기하 가정에 따른 값이며 자동 측정값이 아니다.
거리·입사각 입력으로 검출점을 만들거나 ROI를 자동으로 만들어 맞추지 않는다.

- `scans.csv`에는 정상 0점 frame도 행이 남는다. 점이 없으므로 `points.csv`에는
  그 frame의 점 행이 없다. SNR 결측은 빈 칸이며 0 dB로 치환하지 않는다.
- 원본 PointCloud2에는 `x,y,z,intensity,doppler,snr,noise`가 있다.
  기존 RViz용 `intensity`는 `snr`과 같은 dB 값이다.
- points.csv의 x_m/y_m/z_m은 raw iwr6843_frame 값이다. mounted_x_m/y_m/z_m은
  viewer와 동일한 장착 변환을 적용한 값이며, 좌표계 이름은 trial.json의
  mounted_point_frame에 기록한다. 장착 설정·SHA-256·미보정 상태도 보존한다.
- 일반 rosbag recorder의 기본 경로도 `daq_ws/outputs/daq_bags`로 변경했다.
  Radar trial의 기본 경로는 `daq_ws/outputs/radar_trials`이며 두 폴더는 git 제외 대상이다.
- `/iwr6843_scan_info`에는 frame 번호·수신 시각·누락·cfg hash·시험 조건이 있다.
  기존 `bag_record.launch.py`의 기본 기록 목록에도 추가했다.
- timestamp는 **host에서 UART를 읽은 ROS 시각**이다. 센서 측정 시각이 아니며,
  한 번의 read에 포함된 여러 frame은 같은 수신 시각을 가질 수 있다.
  서로 다른 scan은 hardware frame 번호로 구분한다.
- frame 번호 누락, 파싱 오류, sequence 재시작, queue overflow는 summary에 남는다.
  수신하지 못한 frame을 0점 frame으로 만들어 검출률 분모에 넣지 않는다.
  통신이 끊기면 이전 점을 새 timestamp로 재발행하지 않고 시험을 중단한다.
- 시작 실패·사용자 중단 시에도 가능한 결과를 남긴다. `duration_completed`를
  확인한다. `diagnostics`에 이상이 있으면 정상 수신한 scan만으로 나온 비율이라는
  제한을 확인해야 한다. 마지막에 처리되지 않은 queue/불완전 packet도 표시한다.
- raw UART는 기록 구간의 경계에서 일부 frame만 포함할 수 있다. CSV는 완성·파싱된
  정상 frame만 포함하며 요청 시간 뒤의 데이터는 CSV에서 제외된다.

## 유리 내부 검출 통계 사용

선택 JSON의 예시는 `src/daq_bringup/config/iwr6843isk/glass_roi.example.json`이다.
**예시 좌표는 현장 측정값이 아니다.** 자동으로 사용하지 않으며 실제 유리의
위치·자세·분석 영역을 raw `iwr6843_frame` 좌표로 지정해야 한다.
센서 위치나 자세를 바꾸면 ROI도 현재 Radar 좌표계에 맞아야 한다.

```json
{
  "frame_id": "iwr6843_frame",
  "center_m": [0.0, 2.0, 0.0],
  "normal": [0.0, 1.0, 0.0],
  "horizontal_axis": [0.0, 0.0, 1.0],
  "width_m": 1.0,
  "height_m": 1.0,
  "plane_tolerance_m": 0.2,
  "edge_exclusion_m": 0.1
}
```

center/normal이 평면을, horizontal_axis가 평면 안 가로 방향을 정의한다.
가로축의 법선 성분은 제거하고 정규화한다. 세로축은 두 축의 외적으로 정한다.
width/height는 제외 전 영역 크기이며, slab 허용 오차와 안쪽 경계 제외 폭은
각각 plane_tolerance_m, edge_exclusion_m이다. 예시 오차 폭은 미검증 초기값이다.

```bash
ros2 launch daq_bringup radar_distance_angle_test.launch.py \
  plane_distance_m:=1.0 incidence_angle_deg:=30.0 \
  trial_id:=d1_a30_repeat2 duration_s:=30.0 \
  roi_file:=/absolute/path/to/measured_glass_roi.json rviz:=true
```

- ROI에 점이 하나 이상 있는 scan을 지지 scan으로 센다.
- summary의 `roi_support_rate`는 지지 scan 수 / 정상 수신한 서로 다른 scan 수다.
- SNR은 먼저 scan별 검출점 중앙값을 구하고, summary에는 그 중앙값들의 평균을
  출력한다. 전체 점 기준과 ROI 점 기준을 따로 쓴다. 해당 점이 없는 scan은 SNR
  평균에서 제외되므로 검출 비율과 같이 읽어야 한다.
- 큰 유리의 반사 위치가 조준점에서 이동할 수 있으므로 기준 거리 주변만 자르지
  말고 유한 유리 내부와 실제 점 위치를 함께 확인한다. 같은 평면의 창틀·다중경로를
  완전히 배제했다는 보장은 없으며, 유리 제거 대조군의 오검출률도 아니다.

## D455 동시 기록

필요하면 기존 D455 launch를 별도 터미널에서 시작하고, 시험 중 다른 터미널에서
기존 `bag_record.launch.py bag_name:=<식별자>`를 실행한다. 시험 launch는 Radar를
이미 시작하므로 `iwr6843isk.launch.py`를 중복 실행하지 않는다. D455 bag 기록은
별도 시작·종료되므로 Radar trial과 동일한 시간 구간이라고 자동 가정하지 않는다.

## 검증

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/daq_ws/install/setup.bash
cd ~/workspace/daq_ws
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 \
  python3 -m pytest src/daq_iwr6843isk/test -q
```

별도 ROS domain에서 합성 packet·가상 UART·가상 시리얼 포트로 검사한다.
실제 `/dev/ttyUSB*`를 열지 않는다. 실제 장비의 firmware 호환성, CLI 응답,
0점 frame 송신 및 SNR 수신은 현장 확인이 남아 있다.

2026-10-22 구현 검증 결과:

- `colcon build --symlink-install`: 두 패키지 빌드 성공.
- `colcon test --packages-select daq_iwr6843isk --event-handlers console_direct+`:
  28개 테스트 통과(domain 187, 가상 UART 사용).
- packet 분할·동시 수신, 0점, side-info 순서·누락, 잘못된 packet 재동기화,
  frame 중복·번호 누락·wrap/restart, ROI 내부/경계, 입력 조건 저장 왕복을 검사했다.
- 실제 ROS publisher/subscriber와 가상 UART로 이전 점 재발행 없음,
  정상 0점 발행, 무수신 timeout과 0점 구분을 확인했다.
- pseudo terminal 두 개를 사용해 설치된 launch를 별도 프로세스로 실행하고,
  인자 전달→cfg 응답 확인→CSV/원본 기록→지정 시간 자동 종료까지 확인했다.
- `compileall`, `git diff --check` 통과. 실제 센서·비행 검증은 포함하지 않았다.
- viewer 추가 검증: 현재 장착축의 변환, XYZ/SNR 라벨, SNR 결측과 0 dB 구분,
  정상 0점/무수신 시 표시 제거, raw·mounted CSV와 장착 YAML 저장을 확인했다.
- 실제 RViz를 합성 3점으로 실행해 장착 좌표·점별 XYZ/SNR 라벨과 Global Status OK를
  화면으로 확인했다. 캡처는 `outputs/qa/radar_viewer_xyz_snr.png`이며 센서 실측이 아니다.
- 기본 저장 위치를 `daq_ws/outputs` 아래로 변경했다. 사용자 요청으로 종료된
  home의 `/home/hoyoung/daq_trials` 폴더(약 536 KB, 시행 1개)는 삭제했다.

### RViz 표 표시 개선 검증

2026-10-22: 점 옆 XYZ/SNR 장문 라벨을 번호로 바꾸고, dock형 Radar live values
패널을 추가했다. 파일럿 파일의 첫 프레임 13점을 별도 domain 189에서 재생해
좌표·거리·SNR 표, 센서 원점과 축, Global Status OK를 확인했다.
`outputs/qa/radar_live_table.png`는 저장 데이터 재생 화면이다.
송신을 중단한 뒤 관측점·표는 지워지고 장착 원점만 남는 화면은
`outputs/qa/radar_live_table_stale.png`다. 실제 센서를 재실행하지 않았다.
수정 후 빌드 성공 및 자동 테스트 28개 통과.

### 다음 예비시험: 기본 / 수직각 축소 / 기본

배치: 레이더 원점에서 유리까지 수직거리 1 m, 정면 0도, 같은 센서 위치·자세를
세 기록 동안 유지한다. 원호/평행 이동 방식은 이 시험에서 결정하지 않는다.
현재 실행 중인 센서 launch를 먼저 종료한다. 각 20초씩 순서대로 기록한다.

| 순서 | trial_id | radar_config (src/daq_bringup/config/iwr6843isk/) | raw az / el |
|---|---|---|---|
| 1 | fov_wide_01 | radar.cfg | ±70 / ±30 deg |
| 2 | fov_narrow_01 | radar_fov_vertical15.cfg | ±15 / ±30 deg |
| 3 | fov_wide_02 | radar.cfg | ±70 / ±30 deg |

모두 plane_distance_m:=1.0 incidence_angle_deg:=0.0 off_axis_deg:=0.0
duration_s:=20.0 rviz:=true로 radar_distance_angle_test.launch.py를 실행한다.
새 cfg는 CLI 명령 중 aoaFovCfg 한 줄만 바뀌었음을 검사했다.
±15도는 비교용 초기값이며 유리 ROI의 검증값이 아니다. raw azimuth가 장착상
위·아래 방향에 해당한다. 기본 cfg도 각도 제한이 이미 있으므로 무제한 비교가 아니다.
전체 점 개수 대신 공통 각도 영역의 유리 근처 검출 위치·유지 비율·SNR 및
수신 프레임 손실을 비교한다. 포인트 번호는 scan 간 추적 ID로 쓰지 않는다.
실제 장비에서 좁은 cfg의 CLI 수락 및 효과는 현장 실행 후 확인한다.
