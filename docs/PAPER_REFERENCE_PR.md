# 논문 참조용 센서 취득·측정 계약 보관

Radar취득을hardwareframe기준으로정리하고측정조건·SNR결측·빈scan을보존하는코드와설정·viewer·시험문서를보관한다. 목적은논문작성중구현확인이며촬영원본이나새RF성능결과를업로드하는것은아니다.

- UART packet parsing,hardwareframe중복·누락·0점scan처리와PointCloud/SNR필드보존.
- 거리·입사각·ROI·cfg/장착조건을기록하는취득계약과viewer/RVizpanel.
- 센서·baglaunch와측정/실행문서,촬영·생성물Git제외.

검증:소스2package빌드성공,가상UART를포함한기존회귀28개통과.실제센서/FC는미실행.정량RF검출성능·장착calibration완료를주장하지않는다.

참조태그:`reference/paper-acquisition-2026-11-02`. [상세 범위](PAPER_REFERENCE_SNAPSHOT_2026-11-02.md).
