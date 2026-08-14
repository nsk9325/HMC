#!/usr/bin/env bash
#
# 자율주행 실행 스크립트
#
#   ./run.sh <모드>
#
#   tune        차선 추종 튜닝 (신호등 무시, 즉시 주행)
#   tune-log      "                              + 로그
#   mission     실제 미션 (녹색 신호 대기 → 2바퀴 → 정지)
#   mission-log   "                              + 로그
#   brake       정지 동작만 시험 (랩 카운트 건너뛰고 바로 APPROACH)
#   brake-log     "                              + 로그
#   bench       인지만 확인. 아두이노로 명령을 보내지 않으므로 차량이 움직이지 않음
#   video       번들 영상으로 그래프 동작만 확인 (조향 품질은 무의미)
#
# 아래 '튜닝 값' 블록을 고치고 다시 실행하면 그대로 반영된다.
# 값은 노드 생성 시점에 적용되므로, 기본값으로 잠깐 달리는 구간이 생기지 않는다.
#
# 실행 중 값을 바꾸려면 (재시작 없이 즉시 반영):
#   ros2 param set /mission_planner_node cruise_speed 180
#
set -e

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ══════════════════════════════════════════════════════════════════
#  튜닝 값  —  여기만 고치면 된다
# ══════════════════════════════════════════════════════════════════

# ── 속도 (아두이노 analogWrite 0~255) ───────────────────────────
CRUISE_SPEED=255          # 주행 속도. 실차 상한은 최고속의 70% 부근
APPROACH_SPEED=200        # 정지 목표 접근 속도 = 비례 감속의 최대값.
                          #   MIN_MOVE_SPEED 보다 충분히 커야 감속 곡선이 의미를 가진다
MIN_MOVE_SPEED=40         # 이보다 작은 명령은 바퀴가 돌지 않는다 (실측값).
                          #   너무 낮으면 목표 전에 멈춰 서서 타임아웃까지 대기한다
MAX_REVERSE_SPEED=0       # 목표를 지나쳤을 때 후진으로 되돌리기. 0=비활성.
                          #   타력으로 밀려 자꾸 지나치면 80 정도로 켜 볼 것

# ── 조향 ────────────────────────────────────────────────────────
STEERING_GAIN=0.2         # 방위각[deg] → 조향 단계. 곡선에서 부족하면 올린다
STEERING_SLEW=2           # 한 주기(0.1s)당 조향 변화 제한. 진동이 심하면 낮춘다
LOOKAHEAD_NEAR=30         # 가까운 전방주시점 (위치 오차 보정)
LOOKAHEAD_FAR=70          # 먼 전방주시점 (곡률 선행). ROI 깊이를 넘지 않게
FAR_WEIGHT=0.8            # 0=near만, 1=far만. 언더스티어면 올리고, 흔들리면 내린다
STEERING_SMOOTHING=0.7    # 지수평활 (0~1). 낮을수록 부드럽지만 반응이 늦다
TURN_SLOWDOWN=0.3        # 최대 조향 시 CRUISE_SPEED * (1 - 이 값) 까지 감속

# ── 랩 카운트 (신호등 기준, 색 무관) ────────────────────────────
LAP_LANDMARK_MIN_SCORE=0.5    # 이 점수 미만의 신호등 검출은 무시
LAP_LANDMARK_MIN_HEIGHT=35    # 멀리서 잡히는 신호등을 걸러낸다 (실측 중앙값 46)
LAP_LANDMARK_CLEAR_SEC=3.0    # 이 시간 이상 안 보여야 다음 통과를 받는다.
                              #   같은 신호등이 두 번 세어지면 늘린다
LAP_COOLDOWN_SEC=15.0         # 랩 사이 최소 간격

# ── 정지 ────────────────────────────────────────────────────────
STOPLINE_TRIGGER_Y=430        # 정지선 bbox 하단이 이 y를 넘으면 ARM (640x480 기준)
STOPLINE_LATCH_SEC=8.0        # ARM 유지 시간. 접근 구간을 덮을 만큼
CAR_STOP_BBOX_HEIGHT=230      # 정차 차량 bbox 높이가 이 값이면 정지.
                              #   실측: 210=나란히, 236=지나침
                              #   짧게 서면 올리고, 지나치면 내린다
APPROACH_TIMEOUT_SEC=30.0     # 이 시간 안에 못 서면 실패로 보고 정지

# ── 차선을 잃었을 때 ────────────────────────────────────────────
PATH_TIMEOUT_SEC=1.0      # 경로가 이 시간 이상 끊기면 '차선 상실'로 판정
BLIND_FORWARD_SEC=2.0     # 상실 후 이 시간까지는 조향 0 으로 직진 유지
BLIND_FORWARD_SPEED=90    # 직진 유지 중 속도
                          #   횡단보도에서 매번 멈춰 선다면 BLIND_FORWARD_SEC 를 늘린다

# ── 인지 ────────────────────────────────────────────────────────
THRESHOLD=0.3             # YOLO 검출 임계값. 차선이 자주 끊기면 0.25 로
DEVICE=cuda:0             # CUDA 미설정이면 cpu (약 3Hz, 주행 불가 수준)
SHOW_IMAGE=false          # 실차에서는 false. 디버그 창은 프레임률을 깎는다

# ══════════════════════════════════════════════════════════════════
#  이하 수정 불필요
# ══════════════════════════════════════════════════════════════════

MODE="${1:-}"
if [ -z "$MODE" ]; then
    sed -n '3,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 1
fi

case "$MODE" in
    tune)        USE_SERIAL=true;  SRC=camera; GREEN=false; LAPS=2; LOG=false ;;
    tune-log)    USE_SERIAL=true;  SRC=camera; GREEN=false; LAPS=2; LOG=true  ;;
    mission)     USE_SERIAL=true;  SRC=camera; GREEN=true;  LAPS=2; LOG=false ;;
    mission-log) USE_SERIAL=true;  SRC=camera; GREEN=true;  LAPS=2; LOG=true  ;;
    brake)       USE_SERIAL=true;  SRC=camera; GREEN=false; LAPS=0; LOG=false ;;
    brake-log)   USE_SERIAL=true;  SRC=camera; GREEN=false; LAPS=0; LOG=true  ;;
    bench)       USE_SERIAL=false; SRC=camera; GREEN=false; LAPS=2; LOG=false ;;
    video)       USE_SERIAL=false; SRC=video;  GREEN=false; LAPS=2; LOG=false ;;
    *) echo "알 수 없는 모드: $MODE"; exit 1 ;;
esac

PARAMS_FILE="${WORKSPACE}/src/launch_pkg/config/mission_params.yaml"
cat > "$PARAMS_FILE" <<EOF
# run.sh 가 자동 생성한다. 직접 고치지 말고 run.sh 상단의 변수를 수정할 것.
/**:
  ros__parameters:
    cruise_speed: ${CRUISE_SPEED}
    approach_speed: ${APPROACH_SPEED}
    min_move_speed: ${MIN_MOVE_SPEED}
    max_reverse_speed: ${MAX_REVERSE_SPEED}
    steering_gain: ${STEERING_GAIN}
    steering_slew: ${STEERING_SLEW}
    lookahead_near: ${LOOKAHEAD_NEAR}
    lookahead_far: ${LOOKAHEAD_FAR}
    far_weight: ${FAR_WEIGHT}
    steering_smoothing: ${STEERING_SMOOTHING}
    turn_slowdown: ${TURN_SLOWDOWN}
    lap_landmark_min_score: ${LAP_LANDMARK_MIN_SCORE}
    lap_landmark_min_height: ${LAP_LANDMARK_MIN_HEIGHT}
    lap_landmark_clear_sec: ${LAP_LANDMARK_CLEAR_SEC}
    lap_cooldown_sec: ${LAP_COOLDOWN_SEC}
    stopline_trigger_y: ${STOPLINE_TRIGGER_Y}
    stopline_latch_sec: ${STOPLINE_LATCH_SEC}
    car_stop_bbox_height: ${CAR_STOP_BBOX_HEIGHT}
    approach_timeout_sec: ${APPROACH_TIMEOUT_SEC}
    path_timeout_sec: ${PATH_TIMEOUT_SEC}
    blind_forward_sec: ${BLIND_FORWARD_SEC}
    blind_forward_speed: ${BLIND_FORWARD_SPEED}
EOF

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

echo "─────────────────────────────────────────────"
echo " 모드      : ${MODE}"
echo " 아두이노  : ${USE_SERIAL}   영상: ${SRC}   신호대기: ${GREEN}   바퀴: ${LAPS}   로그: ${LOG}"
echo " 속도      : cruise ${CRUISE_SPEED} / approach ${APPROACH_SPEED} / min ${MIN_MOVE_SPEED}"
echo " 조향      : gain ${STEERING_GAIN} / near ${LOOKAHEAD_NEAR} far ${LOOKAHEAD_FAR} w ${FAR_WEIGHT}"
echo " 정지      : car_h ${CAR_STOP_BBOX_HEIGHT} / latch ${STOPLINE_LATCH_SEC}s"
[ "$USE_SERIAL" = "true" ] && echo " ⚠ 차량이 움직입니다. Ctrl-C 로 정지 명령이 전송됩니다."
echo "─────────────────────────────────────────────"

exec ros2 launch launch_pkg mission.launch.py \
    use_serial:="${USE_SERIAL}" \
    data_source:="${SRC}" \
    wait_for_green:="${GREEN}" \
    target_lap_count:="${LAPS}" \
    debug_log:="${LOG}" \
    show_image:="${SHOW_IMAGE}" \
    device:="${DEVICE}" \
    threshold:="${THRESHOLD}" \
    params_file:="${PARAMS_FILE}"
