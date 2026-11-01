# 논문 참조용 DAQ 코드 스냅샷 — 2026-11-02

이 업로드는 논문 작성 중 센서 취득 코드·설정·측정 계약을 확인하기 위한 보관본이다. 촬영 데이터나 새 물리 실험 결과를 게시하는 작업은 아니다.

- 작업 브랜치: `reference/paper-sensor-acquisition-20261102`
- 참조 태그: `reference/paper-acquisition-2026-11-02`
- 기존 기준: `main`의 `7bdc3a3`. 이 기준에서 작업 브랜치를 만들었다.

## 보관하는 구현

- 완성된 Radar hardware frame마다 한 번 발행하고 정상0점frame도 보존하는 UART/프로토콜 처리.
- frame 번호 기반 중복·누락·역행 진단, SNR/noise side information과 결측값 구분.
- 실제 거리·입사각·off-axis·trial_id·cfg/장착 조건을 남기는 기록 계약과 유한 ROI 지원.
- 장착 추정값을 표시하는 viewer와 RViz panel, stale/empty scan의 이전 점 지우기.
- 센서·bag launch와 QoS, 기록 토픽 설정, 회귀 시험 코드와 관련 문서.

정밀 장착 calibration, 유리 검출 성능, 거리·입사각별 정량 RF 결과를 이번 업로드만으로 입증하지 않는다. SNR 결측을0으로 바꾸거나 ROS timer 발행 횟수를hardware검출률 분모로 사용하지 않는다. 원본frame_id와관측시각을보존한다.

## 이번 확인

2026-11-02 소스에서 2개 package 빌드가 통과했고, 기존 protocol/기록/viewer/가상UART ROS·launch 회귀 **28개가 통과**했다. 실제 카메라·Radar·FC는 실행하지 않았다. 가상 UART 테스트에만 별도 ROS_DOMAIN_ID231을 사용했다.

빌드·검사 생성물과 세부 로컬 감사는 `glass_navigation_ws/outputs/audits/DAQ_UPLOAD_20261102/`에 있고 Git에서 제외한다. 다른 workspace의 검증을 이 저장소의 RF 성능 결과로 취급하지 않는다.

## 업로드 제외

build/install/log/outputs/bags/data/models/.venv,원본bag·동영상·캡처·bytecode를커밋하지않는다. 이스냅샷은코드·설정·시험코드·분석문서만포함한다. 개인공간에서촬영한RGB는올리지않는다.
