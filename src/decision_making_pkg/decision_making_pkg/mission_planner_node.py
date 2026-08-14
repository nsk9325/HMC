"""
mission_planner_node

기존 motion_planner_node의 대체 노드. (원본 파일은 수정하지 않음)

원본은 매 주기마다 즉각 반응하는 if/elif 구조여서, 미션 수행에 필요한 '진행 상태'를
전혀 기억하지 못했다. 본 노드는 명시적인 상태 기계로 미션을 수행한다.

    WAIT_GREEN ──(녹색 신호)──> DRIVING ──(2바퀴 완주)──> APPROACH
                                                            │
                                    (정지선 목격 = ARM, 래치)
                                                            │
                                    (정차 차량 bbox 높이 도달)
                                                            ▼
                                                         STOPPED

원본 대비 변경점:
  1. 녹색 신호 대기 후 출발 (원본에는 출발 게이트 자체가 없음)
  2. 신호등(색 무관) 검출로 랩 카운트
  3. 정지선(Stopline) + 정차 차량(Car) 기반 정지
  4. 비례 조향 (원본은 기울기 부호만 보는 ±7 뱅뱅 제어)
  5. 조향량에 따른 감속
  6. 경로 신호 끊김(staleness) 시 정지
  7. 원본 버그 수정:
     - self.detection_data가 None인 상태에서 .detections 접근 (AttributeError)
     - 적색 신호 분기에서 속도 명령이 갱신되지 않아 이전 값이 유지되던 문제
"""

import math
import os
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# YOLO 클래스 이름 (Roboflow 라벨과 대소문자까지 정확히 일치해야 함)
# 랩 카운트 기준 랜드마크. 신호등은 출발/결승 지점에 있고 색과 무관하게 세야 하므로
# 세 가지 점등 상태를 모두 포함한다.
LAP_LANDMARK_CLASS_NAMES = ('Greenlight', 'Red_light', 'Orange_light')
STOPLINE_CLASS_NAME = 'Stopline'
CAR_CLASS_NAME = 'Car'

# --- 미션 ---
# 완주해야 할 바퀴 수
TARGET_LAP_COUNT = 2

# --- 속도 (Arduino analogWrite 기준 0~255) ---
# 실차 시험 결과 최고 속도의 약 70%가 제어 가능한 상한
CRUISE_SPEED = 240      # 주행 속도
# 정지 목표 접근 속도 (비례 감속의 최대값 = ratio 1.0 일 때의 속도).
# 비례 감속이 의미를 가지려면 MIN_MOVE_SPEED 보다 충분히 커야 한다.
# 90 이면 계산 속도가 h=45 부터 이미 바닥값에 붙어버려 사실상 정속이 된다.
APPROACH_SPEED = 150

# --- 조향 ---ㅇㅇㅇㅇㅇ
MAX_STEERING = 7        # 아두이노 조향 단계 상한 (한쪽 기준)
STEERING_SLEW = 2       # 한 주기당 조향 변화량 제한 (진동 억제)

# 조향 게인. 전방주시점 거리 = index x 1.76px, 조감도 1px ~= 0.234cm (차선폭 256px = 약 60cm)
STEERING_GAIN = 0.2

# 전방주시점을 2개 사용한다.
#   near = 현재 위치 오차를 잡는 항  (반응성)
#   far  = 다가올 곡률을 미리 반영하는 항 (선행성)
# 한 점만 쓰면 이 둘이 뒤섞여, 직선에서 흔들리면서 곡선에서는 언더스티어가 난다.
LOOKAHEAD_NEAR = 30     # 약 13cm 앞
LOOKAHEAD_FAR = 70      # 약 35cm 앞
# 0=near만, 1=far만. 시뮬레이션 결과 (직선 잡음 / 곡선 응답):
#   이전 단일60 무평활 : 부호반전 6회, 표준편차 0.68 / 급곡선 6
#   w=0.5              : 0회, 0.53 / 5   <- 곡선 응답이 오히려 줄어듦
#   w=0.8              : 0회, 0.42 / 7   <- 직선·곡선 모두 개선
#   w=1.0              : 0회, 0.31 / 7   <- 가장 부드럽지만 위치오차 보정이 느려짐
FAR_WEIGHT = 0.8

# 조향 출력 지수평활 계수 (0~1). 낮을수록 부드럽지만 반응이 늦다.
# 차선 마스크의 프레임별 잡음이 그대로 조향에 실리는 것을 막는다.
STEERING_SMOOTHING = 0.7

# 조향이 클수록 감속: 최대 조향 시 CRUISE_SPEED * (1 - TURN_SLOWDOWN)
TURN_SLOWDOWN = 0.3

# --- 랩 카운트 ---
#
# 랜드마크를 횡단보도에서 '신호등'으로 바꿨다.
# 횡단보도는 정지 구역과 가까워 2바퀴째 카운트가 이르게 발생, 조기 정지를 유발했다.
# 신호등은 출발/결승 지점에 있어 한 바퀴에 한 번만 지나가고, 검출도 강하다
# (정지 구역 실측 conf 0.67~0.98).
#
# ⚠️ 신호등 bbox 높이는 근접 시 화면 상단에서 잘리므로(실측 상단y=0) 거리 척도로 쓸 수 없다.
#    따라서 '보이는가'를 기준으로 하고, 멀리서 잡히는 것을 걸러내는 용도로만 높이를 쓴다.
#    train 300장 실측: 신호등 검출 17%, 높이 중앙값 46 (p10=23, 최대 58).
LAP_LANDMARK_MIN_SCORE = 0.5
LAP_LANDMARK_MIN_HEIGHT = 35

# 랜드마크가 이 시간 이상 '연속으로' 안 보여야 다음 통과를 받을 준비를 한다.
#
# 2026-08-13 실주행 로그(mission_1786614496.csv)에서:
#   단 1프레임(0.1s) 미검출로 래치가 풀려, 같은 랜드마크를 5초 뒤 두 번째로 카운트했다.
# 한 프레임 끊김으로 래치를 풀면 안 된다.
LAP_LANDMARK_CLEAR_SEC = 3.0

# 한 번의 통과가 여러 번 세어지지 않도록 하는 최소 간격.
# 실제 한 바퀴는 수십 초이므로 넉넉하게 잡는다.
LAP_COOLDOWN_SEC = 15.0

# --- 정지 판정 ---
#
# 2단계 구조. 두 조건을 '같은 프레임에서' 요구하지 않는 것이 핵심이다.
#
#   1) ARM   : 정지선이 범퍼에 닿는 것을 한 번이라도 보면 래치한다.
#              "트랙의 올바른 위치에 왔다"는 확인용일 뿐, 정지 트리거가 아니다.
#   2) STOP  : 래치된 상태에서 정차 차량의 bbox 높이가 임계값에 도달하면 정지.
#
# 왜 이렇게 바꿨는가 (img/stops/ 실측, conf 0.25):
#   661: Car h=176, Stopline 하단 y=477 (conf 0.92)  <- 정지선이 보이는 유일한 순간
#   411: Car h=210, Stopline 미검출                   <- 이미 차체 아래로 지나감
#   380: Car h=236 (지나쳐서 후진 중)
#   49 : Car h=251
# 두 조건이 겹치는 구간이 사실상 한 순간뿐이라, 그 프레임에서 정지선을 놓치면
# 영원히 정지하지 못하고 한 바퀴를 더 돌게 된다. 래치는 이 우연 의존을 없앤다.
#
# 또한 정지 트리거를 '시간(CREEP)'에서 'Car bbox 높이'로 바꾸면 거리 기반이 되어
# 배터리 전압이 떨어져도 같은 위치에 선다. (같은 PWM = 더 느린 속도 = 더 짧은 전진)
# Car 는 모든 정지 프레임에서 conf 0.90~0.95 로 잡히고, 차량이 화면 좌측으로 잘려도
# 지붕~휠 림의 세로 길이는 남으므로 높이는 계속 유효하다.

# 정지선 bbox 하단이 이 y를 넘으면 ARM (원본 640x480 좌표 기준)
STOPLINE_TRIGGER_Y = 430
# ARM 상태를 유지하는 시간. 접근 구간을 덮되 다음 바퀴로 새지 않을 만큼.
STOPLINE_LATCH_SEC = 8.0
# ARM 상태에서 이 높이에 도달하면 정지. 210=나란히, 236=지나침 이므로 그 사이.
# 차량은 제동이 아니라 타력 주행으로 멈추므로, 밀리는 만큼 낮춰 잡는다.
CAR_STOP_BBOX_HEIGHT = 205
# APPROACH 안전장치: 이 시간 안에 정지하지 못하면 실패로 보고 정지한다.
# (미정지 상태로 계속 주행하는 것보다 서는 편이 안전하다)
APPROACH_TIMEOUT_SEC = 30.0

# --- 정지 목표 접근 시 비례 감속 ---
#
# ARM 이후에는 남은 거리에 비례해 감속하여, 목표에 '거의 정지한 상태'로 도달한다.
# 차량은 제동이 아니라 타력으로 멈추므로, 도달 속도를 낮추는 것이 곧 정지 정밀도다.
#
#   ratio = (car_stop_bbox_height - 현재 h) / car_stop_bbox_height
#   속도  = APPROACH_SPEED * ratio
#
# bbox 높이는 거리에 반비례(h = k/Z)하므로 이 식이 남은 거리에 정확히 비례하지는
# 않지만, 먼 곳에서 더 일찍 감속하는 보수적인 곡선이라 안전한 방향이다.
#
# ⚠️ MIN_MOVE_SPEED 는 반드시 실측할 것.
#    이 값보다 작은 명령은 정지 마찰을 못 이겨 바퀴가 아예 돌지 않는다.
#    바닥값이 없으면 목표(h=205)에 닿기 전에 멈춰 서고,
#    STOPPED 조건을 만족하지 못한 채 APPROACH 타임아웃까지 대기하게 된다.
#
#    data_collection 도구(w 1회 = +10)로 실측한 결과 1~2회 만으로 크롤이 가능했으므로 20.
#    주의: 이 값은 '정지 상태에서 출발'이 아니라 '이미 움직이는 상태를 유지'하는
#    기준이다. 감속 구간에서는 운동마찰이 적용되므로 이 값이 맞다.
MIN_MOVE_SPEED = 20

# 목표를 지나쳤을 때(h > 목표) 후진으로 되돌릴지 여부.
# 0 이면 후진 없이 정지만 한다. 실차에서 검증 후 켤 것 (예: 80).
MAX_REVERSE_SPEED = 0

# --- 안전 ---
# 경로가 이 시간 이상 갱신되지 않으면 '차선을 잃었다'고 판정
PATH_TIMEOUT_SEC = 1.0

# 차선을 잃은 직후의 대응.
# 즉시 정지하면 횡단보도처럼 마스크가 잠깐 끊기는 구간에서 매번 멈춰 선다.
# 반대로 마지막 조향을 유지하면 잘못된 조향값(예: -7)이 그대로 이어져 차선을 이탈한다.
# 따라서 '조향 0 으로 잠시 직진'한 뒤, 그래도 회복되지 않으면 정지한다.
BLIND_FORWARD_SEC = 2.0     # 이 시간까지는 직진 유지
BLIND_FORWARD_SPEED = 90    # 직진 유지 중 속도
# 주행 중에도 적색 신호에 정지할지 여부.
# 본 미션의 신호등은 '출발 게이트'이므로 기본값은 False.
STOP_ON_RED_WHILE_DRIVING = False

# 녹색 신호를 기다렸다가 출발할지 여부.
# False로 두면 WAIT_GREEN을 건너뛰고 즉시 DRIVING 상태로 시작한다.
# 차선 추종/조향 튜닝처럼 신호등이 없는 곳에서 시험할 때 사용.
# ⚠️ 실제 미션 주행에서는 반드시 True 여야 한다.
WAIT_FOR_GREEN = True

# --- 디버그 ---
# True로 두면 매 주기의 상태/입력/출력을 CSV로 남긴다.
# 왜 출발하지 않았는지, 어느 클래스가 몇 점으로 잡혔는지를 사후에 확인할 수 있다.
DEBUG_LOG = False
DEBUG_LOG_DIR = 'debug_logs'

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1
#----------------------------------------------


class MissionState(Enum):
    WAIT_GREEN = 'WAIT_GREEN'
    DRIVING = 'DRIVING'
    APPROACH = 'APPROACH'
    STOPPED = 'STOPPED'


class MissionPlanningNode(Node):
    def __init__(self):
        super().__init__('mission_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        self.timer_period = self.declare_parameter('timer', TIMER).value

        # 미션/제어 파라미터
        self.target_lap_count = self.declare_parameter('target_lap_count', TARGET_LAP_COUNT).value
        self.cruise_speed = self.declare_parameter('cruise_speed', CRUISE_SPEED).value
        self.approach_speed = self.declare_parameter('approach_speed', APPROACH_SPEED).value
        self.steering_gain = self.declare_parameter('steering_gain', STEERING_GAIN).value
        self.steering_slew = self.declare_parameter('steering_slew', STEERING_SLEW).value
        self.lookahead_near = self.declare_parameter('lookahead_near', LOOKAHEAD_NEAR).value
        self.lookahead_far = self.declare_parameter('lookahead_far', LOOKAHEAD_FAR).value
        self.far_weight = self.declare_parameter('far_weight', FAR_WEIGHT).value
        self.steering_smoothing = self.declare_parameter('steering_smoothing', STEERING_SMOOTHING).value
        self.debug_log = self.declare_parameter('debug_log', DEBUG_LOG).value
        self.debug_log_dir = self.declare_parameter('debug_log_dir', DEBUG_LOG_DIR).value
        self.turn_slowdown = self.declare_parameter('turn_slowdown', TURN_SLOWDOWN).value
        self.lap_cooldown_sec = self.declare_parameter('lap_cooldown_sec', LAP_COOLDOWN_SEC).value
        self.lap_landmark_clear_sec = self.declare_parameter(
            'lap_landmark_clear_sec', LAP_LANDMARK_CLEAR_SEC).value
        self.lap_landmark_min_score = self.declare_parameter(
            'lap_landmark_min_score', LAP_LANDMARK_MIN_SCORE).value
        self.lap_landmark_min_height = self.declare_parameter(
            'lap_landmark_min_height', LAP_LANDMARK_MIN_HEIGHT).value
        self.stopline_trigger_y = self.declare_parameter('stopline_trigger_y', STOPLINE_TRIGGER_Y).value
        self.stopline_latch_sec = self.declare_parameter('stopline_latch_sec', STOPLINE_LATCH_SEC).value
        self.car_stop_bbox_height = self.declare_parameter('car_stop_bbox_height', CAR_STOP_BBOX_HEIGHT).value
        self.approach_timeout_sec = self.declare_parameter('approach_timeout_sec', APPROACH_TIMEOUT_SEC).value
        self.min_move_speed = self.declare_parameter('min_move_speed', MIN_MOVE_SPEED).value
        self.max_reverse_speed = self.declare_parameter('max_reverse_speed', MAX_REVERSE_SPEED).value
        self.path_timeout_sec = self.declare_parameter('path_timeout_sec', PATH_TIMEOUT_SEC).value
        self.blind_forward_sec = self.declare_parameter('blind_forward_sec', BLIND_FORWARD_SEC).value
        self.blind_forward_speed = self.declare_parameter('blind_forward_speed', BLIND_FORWARD_SPEED).value
        self.stop_on_red_while_driving = self.declare_parameter(
            'stop_on_red_while_driving', STOP_ON_RED_WHILE_DRIVING).value
        self.wait_for_green = self.declare_parameter('wait_for_green', WAIT_FOR_GREEN).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 수신 데이터
        self.detection_data = None
        self.path_data = None
        self.path_stamp = None
        self.traffic_light_data = None
        self.lidar_data = None

        # 미션 상태
        self.state = MissionState.WAIT_GREEN if self.wait_for_green else MissionState.DRIVING
        if not self.wait_for_green:
            self.get_logger().warn(
                "wait_for_green=False - 신호등을 무시하고 즉시 주행을 시작합니다. "
                "실제 미션 주행에서는 True로 되돌릴 것.")
        self.lap_count = 0
        self.last_lap_stamp = None
        # 출발 시 차량은 신호등 바로 아래에 있다. 래치를 걸어둔 채로 시작해야
        # 출발하자마자 1바퀴로 세어지지 않는다. 신호등이 시야에서 사라져야 풀린다.
        self.lap_landmark_latched = True
        self.lap_landmark_last_seen = None
        self.stopline_armed_stamp = None  # 정지선을 마지막으로 본 시각 (ARM 래치)
        self.approach_start_stamp = None  # APPROACH 진입 시각
        self.stopline_armed = False       # ARM 여부 (compute_command 에서 참조)

        # 일시정지 (서비스로 토글). 상태 기계는 그대로 두고 명령만 0 으로 만든다.
        self.paused = False
        self.pause_started = None

        # 출력 명령
        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        self.steer_filtered = 0.0     # 지수평활된 조향 (정수 반올림 전)

        # 디버그 로그
        self.log_file = None
        if self.debug_log:
            self.open_debug_log()

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 일시정지 토글 서비스
        self.pause_srv = self.create_service(Trigger, '~/toggle_pause', self.toggle_pause_cb)

        # 실행 중 파라미터 변경을 실제로 반영하기 위한 콜백.
        # declare_parameter(...).value 는 생성 시점에 한 번만 읽으므로,
        # 이 콜백이 없으면 ros2 param set 을 해도 노드 동작은 바뀌지 않는다.
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # 실행 중 변경 가능한 파라미터. 이름과 인스턴스 속성명이 같은 것만 허용한다.
    LIVE_TUNABLE = {
        'cruise_speed', 'approach_speed', 'target_lap_count',
        'steering_gain', 'steering_slew', 'lookahead_near', 'lookahead_far',
        'far_weight', 'steering_smoothing', 'turn_slowdown',
        'lap_landmark_min_height', 'lap_cooldown_sec', 'lap_landmark_clear_sec',
        'stopline_trigger_y', 'stopline_latch_sec', 'car_stop_bbox_height',
        'approach_timeout_sec', 'min_move_speed', 'max_reverse_speed',
        'path_timeout_sec', 'blind_forward_sec', 'blind_forward_speed',
        'stop_on_red_while_driving',
    }

    def toggle_pause_cb(self, request, response):
        """일시정지 토글. 상태 기계는 유지한 채 구동 명령만 0 으로 만든다."""
        # 서비스 콜백에서 예외가 나면 rclpy 가 spin() 밖으로 전파시켜 노드가 죽는다.
        # 일시정지는 주행 중에 쓰는 기능이므로 절대 노드를 죽여서는 안 된다.
        try:
            now = self.now_sec()
            self.paused = not self.paused

            if self.paused:
                self.pause_started = now
            else:
                # 정지해 있던 시간만큼 타이머 기준시각을 뒤로 민다.
                # 그렇지 않으면 일시정지 중에 APPROACH 타임아웃이나 랩 래치가 만료된다.
                if self.pause_started is not None:
                    held = now - self.pause_started
                    for attr in ('approach_start_stamp', 'stopline_armed_stamp',
                                 'last_lap_stamp', 'lap_landmark_last_seen'):
                        value = getattr(self, attr, None)
                        if value is not None:
                            setattr(self, attr, value + held)
                self.pause_started = None

            state = "일시정지" if self.paused else "재개"
            self.get_logger().warn(f"=== {state} ===")
            response.success = True
            response.message = "paused" if self.paused else "running"
        except Exception as e:
            self.get_logger().error(f"일시정지 토글 처리 중 오류: {e}")
            response.success = False
            response.message = str(e)
        return response

    def on_parameter_change(self, params):
        """ros2 param set 으로 들어온 값을 인스턴스 속성에 실제로 반영한다."""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name in self.LIVE_TUNABLE:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"파라미터 변경: {param.name} = {param.value}")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    # 콜백
    # ------------------------------------------------------------------ #
    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
        self.path_stamp = self.now_sec()

    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    # ------------------------------------------------------------------ #
    # 보조 함수
    # ------------------------------------------------------------------ #
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ #
    # 디버그 로그
    # ------------------------------------------------------------------ #
    def open_debug_log(self):
        """주행 1회당 CSV 파일 1개. 파일명에 쓸 시각은 ROS 시계에서 가져온다."""
        try:
            os.makedirs(self.debug_log_dir, exist_ok=True)
            path = os.path.join(self.debug_log_dir, f"mission_{int(self.now_sec())}.csv")
            self.log_file = open(path, 'w', buffering=1)
            self.log_file.write(
                "t,state,lap,traffic_light,steering,left_speed,right_speed,"
                "path_age,n_det,detections\n")
            self.get_logger().info(f"디버그 로그: {os.path.abspath(path)}")
        except Exception as e:
            self.get_logger().error(f"디버그 로그를 열 수 없음: {e}")
            self.log_file = None

    def write_debug_log(self):
        if self.log_file is None:
            return
        dets = self.detections()
        # 클래스별 최고 점수만 압축해서 기록
        best = {}
        for d in dets:
            if d.score > best.get(d.class_name, (0.0,))[0]:
                best[d.class_name] = (d.score, d.bbox.size.y, self.bbox_bottom_y(d))
        # 클래스:점수:높이:하단y  - Car 높이와 Stopline 하단 위치를 사후 확인할 수 있어야 한다
        summary = ' '.join(f"{k}:{v[0]:.2f}:{v[1]:.0f}:{v[2]:.0f}" for k, v in sorted(best.items()))
        age = (self.now_sec() - self.path_stamp) if self.path_stamp is not None else -1.0
        try:
            self.log_file.write(
                f"{self.now_sec():.2f},{self.state.value},{self.lap_count},"
                f"{self.traffic_light_color()},{self.steering_command},"
                f"{self.left_speed_command},{self.right_speed_command},"
                f"{age:.2f},{len(dets)},{summary}\n")
        except Exception:
            pass

    def close_debug_log(self):
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

    def detections(self):
        """수신된 검출 결과를 안전하게 반환 (원본의 None 역참조 버그 방지)."""
        if self.detection_data is None:
            return []
        return self.detection_data.detections

    @staticmethod
    def bbox_bottom_y(detection) -> float:
        return detection.bbox.center.position.y + detection.bbox.size.y / 2

    def find_largest(self, class_name):
        """해당 클래스 중 bbox 높이가 가장 큰 검출을 반환.

        Car 처럼 '가장 가까운 것'을 원할 때는 이쪽을 쓴다. find_class 는 bbox 하단이
        가장 아래인 것을 고르므로, 배경에 다른 차량이 있으면 엉뚱한 것을 집을 수 있다.
        """
        best = None
        for detection in self.detections():
            if detection.class_name != class_name:
                continue
            if best is None or detection.bbox.size.y > best.bbox.size.y:
                best = detection
        return best

    def find_class(self, class_name):
        """해당 클래스 중 bbox가 가장 아래(=가장 가까움)에 있는 검출을 반환."""
        best = None
        for detection in self.detections():
            if detection.class_name != class_name:
                continue
            if best is None or self.bbox_bottom_y(detection) > self.bbox_bottom_y(best):
                best = detection
        return best

    def traffic_light_color(self) -> str:
        if self.traffic_light_data is None:
            return 'None'
        return self.traffic_light_data.data

    def obstacle_detected(self) -> bool:
        return self.lidar_data is not None and self.lidar_data.data is True

    def path_is_fresh(self) -> bool:
        if self.path_data is None or self.path_stamp is None:
            return False
        return (self.now_sec() - self.path_stamp) <= self.path_timeout_sec

    # ------------------------------------------------------------------ #
    # 조향 계산
    # ------------------------------------------------------------------ #
    def bearing_to(self, origin, lookahead):
        """경로 끝(차량)에서 lookahead 만큼 앞의 점을 향하는 방위각[deg]. 계산 불가 시 None."""
        idx = max(0, len(self.path_data) - 1 - int(lookahead))
        target = self.path_data[idx]
        dx = target[0] - origin[0]      # 우측(+) / 좌측(-) 편차
        dy = origin[1] - target[1]      # 전방 거리 (y는 아래로 증가하므로 부호 반전)
        if dy <= 0:
            return None
        return math.degrees(math.atan2(dx, dy))

    def compute_steering(self) -> int:
        """
        경로 위의 전방주시점(lookahead point)을 향하는 방위각으로 조향값을 계산한다.

        path_planner가 경로의 마지막 점으로 차량 앞범퍼 중심을 넣어주므로,
        path_data[-1]이 곧 차량 기준점이 된다. (y가 클수록 차량에 가까움)

        원본은 DMFL.calculate_slope_between_points를 사용했으나, 이 함수는 두 점의 y가
        같을 때 문자열 'inf'를 반환하여 비교 연산에서 TypeError를 일으킨다.
        atan2는 해당 경우를 자연스럽게 처리하므로 직접 계산한다.
        """
        if not self.path_is_fresh() or len(self.path_data) < 2:
            # 마지막 조향을 유지하면 잘못된 값이 그대로 이어지므로 직진을 반환한다.
            return 0

        origin = self.path_data[-1]
        a_near = self.bearing_to(origin, self.lookahead_near)
        a_far = self.bearing_to(origin, self.lookahead_far)

        if a_near is None and a_far is None:
            return self.steering_command
        if a_near is None:
            a_near = a_far
        if a_far is None:
            a_far = a_near

        # near(위치 오차) + far(곡률 선행)의 가중 평균.
        # 두 점을 쓰면 직선에서의 흔들림과 곡선에서의 언더스티어를 따로 조절할 수 있다.
        angle_deg = (1.0 - self.far_weight) * a_near + self.far_weight * a_far

        # 지수평활: 프레임별 마스크 잡음이 조향에 그대로 실리는 것을 막는다.
        raw = self.steering_gain * angle_deg
        self.steer_filtered = (self.steering_smoothing * raw
                               + (1.0 - self.steering_smoothing) * self.steer_filtered)

        # 조향 부호 규약: + = 우조향 (driving.ino의 angle 부호와 일치)
        desired = int(round(self.steer_filtered))
        desired = max(-MAX_STEERING, min(MAX_STEERING, desired))

        # 급격한 조향 변화를 제한하여 좌우 진동을 억제한다.
        # 차선 여유가 편측 8~14cm뿐이므로 진동 자체가 실격 요인이 된다.
        delta = desired - self.steering_command
        if delta > self.steering_slew:
            desired = self.steering_command + self.steering_slew
        elif delta < -self.steering_slew:
            desired = self.steering_command - self.steering_slew

        return desired

    def approach_speed_for(self, car) -> int:
        """
        ARM 이후 정지 목표까지 남은 거리에 비례해 감속한 속도를 반환한다.
        목표가 보이지 않으면 기본 접근 속도를 그대로 쓴다.
        """
        if car is None:
            return self.approach_speed

        hd = float(self.car_stop_bbox_height)
        ratio = (hd - car.bbox.size.y) / hd
        speed = self.approach_speed * ratio

        # 상한/하한
        speed = max(-float(self.max_reverse_speed), min(float(self.approach_speed), speed))

        # 정지 마찰 보정: 계산값이 너무 작으면 바퀴가 돌지 않으므로 바닥값을 준다
        if 0.0 < speed < self.min_move_speed:
            speed = float(self.min_move_speed)
        elif -self.min_move_speed < speed < 0.0:
            speed = -float(self.min_move_speed)

        return int(round(speed))

    def speed_for(self, base_speed: int, steering: int) -> int:
        """조향량이 클수록 감속."""
        ratio = abs(steering) / float(MAX_STEERING)
        return int(round(base_speed * (1.0 - self.turn_slowdown * ratio)))

    # ------------------------------------------------------------------ #
    # 랩 카운트
    # ------------------------------------------------------------------ #
    def find_lap_landmark(self):
        """랩 카운트 기준이 되는 신호등(색 무관) 중 가장 큰 검출을 반환."""
        best = None
        for detection in self.detections():
            if detection.class_name not in LAP_LANDMARK_CLASS_NAMES:
                continue
            if detection.score < self.lap_landmark_min_score:
                continue
            if detection.bbox.size.y < self.lap_landmark_min_height:
                continue
            if best is None or detection.bbox.size.y > best.bbox.size.y:
                best = detection
        return best

    def update_lap_count(self):
        """
        신호등을 지나칠 때마다 한 바퀴로 센다.

        출발 지점이 곧 신호등 아래이므로 래치를 건 상태로 시작한다.
        즉 '신호등이 사라졌다가 다시 나타날 때'만 카운트되며,
        출발 직후의 신호등 목격은 세지 않는다.
        """
        now = self.now_sec()
        landmark = self.find_lap_landmark()

        if landmark is not None:
            self.lap_landmark_last_seen = now
        elif (self.lap_landmark_latched
              and self.lap_landmark_last_seen is not None
              and (now - self.lap_landmark_last_seen) >= self.lap_landmark_clear_sec):
            # 확실히 시야에서 벗어났을 때만 래치를 푼다.
            # 단발성 미검출로 풀면 같은 신호등을 두 번 세게 된다.
            self.lap_landmark_latched = False

        if landmark is None or self.lap_landmark_latched:
            return

        if self.last_lap_stamp is not None and (now - self.last_lap_stamp) < self.lap_cooldown_sec:
            return

        self.lap_count += 1
        self.last_lap_stamp = now
        self.lap_landmark_latched = True
        self.get_logger().info(
            f"LAP {self.lap_count}/{self.target_lap_count} 완료 "
            f"({landmark.class_name} h={landmark.bbox.size.y:.0f})")

    # ------------------------------------------------------------------ #
    # 상태 전이
    # ------------------------------------------------------------------ #
    def update_state(self):
        if self.state is MissionState.WAIT_GREEN:
            if self.traffic_light_color() == 'Green':
                self.get_logger().info("녹색 신호 확인 - 출발")
                self.state = MissionState.DRIVING

        elif self.state is MissionState.DRIVING:
            self.update_lap_count()
            if self.lap_count >= self.target_lap_count:
                self.get_logger().info("2바퀴 완주 - 정지 목표 접근 시작")
                self.state = MissionState.APPROACH

        elif self.state is MissionState.APPROACH:
            now = self.now_sec()
            if self.approach_start_stamp is None:
                self.approach_start_stamp = now

            # 1) ARM: 정지선을 한 번이라도 범퍼 근처에서 보면 래치한다.
            stopline = self.find_class(STOPLINE_CLASS_NAME)
            if (stopline is not None
                    and self.bbox_bottom_y(stopline) >= self.stopline_trigger_y):
                if self.stopline_armed_stamp is None:
                    self.get_logger().info("정지선 확인 - 정지 준비(ARM)")
                self.stopline_armed_stamp = now

            armed = (self.stopline_armed_stamp is not None
                     and (now - self.stopline_armed_stamp) <= self.stopline_latch_sec)
            self.stopline_armed = armed

            # 2) STOP: ARM 상태에서 정차 차량이 충분히 가까워지면 정지.
            #    Car 는 배경 차량과 섞일 수 있으므로 '가장 큰' 것을 고른다.
            car = self.find_largest(CAR_CLASS_NAME)
            if armed and car is not None and car.bbox.size.y >= self.car_stop_bbox_height:
                self.get_logger().info(
                    f"정지 - 정차 차량 bbox 높이 {car.bbox.size.y:.0f} "
                    f"(임계 {self.car_stop_bbox_height})")
                self.state = MissionState.STOPPED

            # 안전장치: 제한 시간 안에 정지하지 못하면 계속 도는 대신 선다.
            elif (now - self.approach_start_stamp) > self.approach_timeout_sec:
                self.get_logger().warn(
                    f"APPROACH {self.approach_timeout_sec}s 초과 - 정지 조건 미충족, 정지")
                self.state = MissionState.STOPPED

    # ------------------------------------------------------------------ #
    # 명령 생성
    # ------------------------------------------------------------------ #
    def compute_command(self):
        # 라이다 장애물은 모든 주행 상태에 우선하는 비상 정지
        if self.obstacle_detected():
            return 0, 0, 0

        # 일시정지: 상태는 유지하고 구동만 멈춘다
        if self.paused:
            return 0, 0, 0

        if self.state in (MissionState.WAIT_GREEN, MissionState.STOPPED):
            return 0, 0, 0

        # DRIVING / APPROACH : 차선 추종
        if self.stop_on_red_while_driving and self.traffic_light_color() == 'Red':
            return 0, 0, 0

        if not self.path_is_fresh():
            age = (self.now_sec() - self.path_stamp) if self.path_stamp is not None else float('inf')
            # 복귀 시 평활 필터가 옛 값에서 튀지 않도록 상태도 함께 0으로 맞춘다
            self.steer_filtered = 0.0
            if age <= (self.path_timeout_sec + self.blind_forward_sec):
                self.get_logger().warn(
                    "경로 신호 없음 - 조향 0 으로 직진 유지", throttle_duration_sec=1.0)
                return 0, self.blind_forward_speed, self.blind_forward_speed
            self.get_logger().warn("경로 신호 없음 지속 - 정지", throttle_duration_sec=1.0)
            return 0, 0, 0

        steering = self.compute_steering()

        if self.state is MissionState.APPROACH and self.stopline_armed:
            # ARM 이후에는 정지 목표까지의 거리에 비례해 감속한다.
            # 조향에 따른 감속은 적용하지 않는다 (감속 요인이 두 번 곱해지는 것을 방지)
            speed = self.approach_speed_for(self.find_largest(CAR_CLASS_NAME))
            return steering, speed, speed

        base_speed = self.cruise_speed if self.state is MissionState.DRIVING else self.approach_speed
        speed = self.speed_for(base_speed, steering)
        return steering, speed, speed

    # ------------------------------------------------------------------ #
    # 메인 루프
    # ------------------------------------------------------------------ #
    def timer_callback(self):
        self.update_state()

        steering, left_speed, right_speed = self.compute_command()

        self.steering_command = steering
        self.left_speed_command = left_speed
        self.right_speed_command = right_speed

        self.get_logger().info(
            f"[{self.state.value}{'|PAUSED' if self.paused else ''}] "
            f"lap: {self.lap_count}/{self.target_lap_count}, "
            f"steering: {self.steering_command}, "
            f"left_speed: {self.left_speed_command}, "
            f"right_speed: {self.right_speed_command}")

        self.write_debug_log()

        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.close_debug_log()
        node.destroy_node()
        # Ctrl-C로 종료되는 경우 rclpy의 시그널 핸들러가 이미 context를 종료시킨 뒤이므로,
        # 여기서 다시 shutdown()을 호출하면 RCLError가 발생한다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
