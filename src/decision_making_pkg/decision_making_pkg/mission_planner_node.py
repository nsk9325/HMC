"""
mission_planner_node

기존 motion_planner_node의 대체 노드. (원본 파일은 수정하지 않음)

원본은 매 주기마다 즉각 반응하는 if/elif 구조여서, 미션 수행에 필요한 '진행 상태'를
전혀 기억하지 못했다. 본 노드는 명시적인 상태 기계로 미션을 수행한다.

    WAIT_GREEN ──(녹색 신호)──> DRIVING ──(2바퀴 완주)──> APPROACH
                                                            │
                                          (정지선이 화면 하단 통과)
                                                            ▼
                                                          CREEP ──> STOPPED

원본 대비 변경점:
  1. 녹색 신호 대기 후 출발 (원본에는 출발 게이트 자체가 없음)
  2. 횡단보도(Crosswalk) 검출로 랩 카운트
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
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

# YOLO 클래스 이름 (Roboflow 라벨과 대소문자까지 정확히 일치해야 함)
CROSSWALK_CLASS_NAME = 'Crosswalk'
STOPLINE_CLASS_NAME = 'Stopline'
CAR_CLASS_NAME = 'Car'

# --- 미션 ---
# 완주해야 할 바퀴 수
TARGET_LAP_COUNT = 2

# --- 속도 (Arduino analogWrite 기준 0~255) ---
# 실차 시험 결과 최고 속도의 약 70%가 제어 가능한 상한
CRUISE_SPEED = 175      # 주행 속도
APPROACH_SPEED = 90     # 정지 목표 접근 속도
CREEP_SPEED = 70        # 정지선을 차체 아래로 보내기 위한 미소 전진 속도

# --- 조향 ---
MAX_STEERING = 7        # 아두이노 조향 단계 상한 (한쪽 기준)
STEERING_SLEW = 2       # 한 주기당 조향 변화량 제한 (진동 억제)

# 조향 게인. 전방주시점 거리 = index x 1.76px, 조감도 1px ~= 0.234cm (차선폭 256px = 약 60cm)
STEERING_GAIN = 0.30

# 전방주시점을 2개 사용한다.
#   near = 현재 위치 오차를 잡는 항  (반응성)
#   far  = 다가올 곡률을 미리 반영하는 항 (선행성)
# 한 점만 쓰면 이 둘이 뒤섞여, 직선에서 흔들리면서 곡선에서는 언더스티어가 난다.
LOOKAHEAD_NEAR = 30     # 약 13cm 앞
LOOKAHEAD_FAR = 85      # 약 35cm 앞
# 0=near만, 1=far만. 시뮬레이션 결과 (직선 잡음 / 곡선 응답):
#   이전 단일60 무평활 : 부호반전 6회, 표준편차 0.68 / 급곡선 6
#   w=0.5              : 0회, 0.53 / 5   <- 곡선 응답이 오히려 줄어듦
#   w=0.8              : 0회, 0.42 / 7   <- 직선·곡선 모두 개선
#   w=1.0              : 0회, 0.31 / 7   <- 가장 부드럽지만 위치오차 보정이 느려짐
FAR_WEIGHT = 0.8

# 조향 출력 지수평활 계수 (0~1). 낮을수록 부드럽지만 반응이 늦다.
# 차선 마스크의 프레임별 잡음이 그대로 조향에 실리는 것을 막는다.
STEERING_SMOOTHING = 0.5

# 조향이 클수록 감속: 최대 조향 시 CRUISE_SPEED * (1 - TURN_SLOWDOWN)
TURN_SLOWDOWN = 0.45

# --- 랩 카운트 ---
# 횡단보도 bbox 하단이 이 y(px, 640x480 원본 기준)를 넘으면 '통과'로 판정
CROSSWALK_TRIGGER_Y = 300

# 횡단보도가 이 시간 이상 '연속으로' 안 보여야 다음 통과를 받을 준비를 한다.
#
# 2026-08-13 실주행 로그(mission_1786614496.csv)에서:
#   t+62.2s 에 단 1프레임(0.1s) 미검출이 발생 -> 래치가 풀림
#   t+66.3s 쿨다운 5초가 지나자 '같은' 횡단보도를 2바퀴째로 카운트
#   결과: 5초 만에 2바퀴 완주로 판정하고 APPROACH 진입
# 한 프레임 끊김으로 래치를 풀면 안 된다.
CROSSWALK_CLEAR_SEC = 2.0

# 한 번의 통과가 여러 번 세어지지 않도록 하는 최소 간격.
# 실제 한 바퀴는 수십 초이므로 넉넉하게 잡는다.
LAP_COOLDOWN_SEC = 15.0

# --- 정지 판정 ---
#
# 역할 분담에 주의할 것:
#   Stopline  = 실제 정지 트리거 (정지선이 범퍼에 닿는 순간)
#   Car 높이  = '정지 구역에 들어왔다'는 확인용 게이트일 뿐, 정밀 거리 측정이 아니다
#
# img/stops/ 실측 (신규 모델, 지붕~휠 림 하단 기준):
#   661: Car h=176, Stopline 하단 y=479  <- 정지선이 이미 범퍼에 도달
#   411: Car h=209, Stopline 미검출      <- 정지선은 차체 아래로 사라진 뒤
#   380: Car h=236 (지나쳐서 후진 중)
#   49 : Car h=251
#
# 즉 정지선이 보이는 시점에 Car 높이는 176 뿐이다. 이 값을 210 등으로 잡으면
# 두 조건이 동시에 성립하는 순간이 존재하지 않아 영원히 정지하지 않는다.
# 따라서 게이트는 176보다 낮게 두고, 정밀도는 CREEP_DURATION_SEC으로 맞춘다.
CAR_MIN_BBOX_HEIGHT = 165
# 정지선 bbox 하단이 이 y를 넘으면 곧 차체 아래로 들어간다고 판정
STOPLINE_TRIGGER_Y = 430
# 정지선이 화면에서 사라진 뒤, 차체 중앙에 오도록 전진하는 시간
CREEP_DURATION_SEC = 0.8

# --- 안전 ---
# 경로가 이 시간 이상 갱신되지 않으면 정지 (차선 인식 실패)
PATH_TIMEOUT_SEC = 1.0
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
    CREEP = 'CREEP'
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
        self.creep_speed = self.declare_parameter('creep_speed', CREEP_SPEED).value
        self.steering_gain = self.declare_parameter('steering_gain', STEERING_GAIN).value
        self.steering_slew = self.declare_parameter('steering_slew', STEERING_SLEW).value
        self.lookahead_near = self.declare_parameter('lookahead_near', LOOKAHEAD_NEAR).value
        self.lookahead_far = self.declare_parameter('lookahead_far', LOOKAHEAD_FAR).value
        self.far_weight = self.declare_parameter('far_weight', FAR_WEIGHT).value
        self.steering_smoothing = self.declare_parameter('steering_smoothing', STEERING_SMOOTHING).value
        self.debug_log = self.declare_parameter('debug_log', DEBUG_LOG).value
        self.debug_log_dir = self.declare_parameter('debug_log_dir', DEBUG_LOG_DIR).value
        self.turn_slowdown = self.declare_parameter('turn_slowdown', TURN_SLOWDOWN).value
        self.crosswalk_trigger_y = self.declare_parameter('crosswalk_trigger_y', CROSSWALK_TRIGGER_Y).value
        self.lap_cooldown_sec = self.declare_parameter('lap_cooldown_sec', LAP_COOLDOWN_SEC).value
        self.crosswalk_clear_sec = self.declare_parameter('crosswalk_clear_sec', CROSSWALK_CLEAR_SEC).value
        self.car_min_bbox_height = self.declare_parameter('car_min_bbox_height', CAR_MIN_BBOX_HEIGHT).value
        self.stopline_trigger_y = self.declare_parameter('stopline_trigger_y', STOPLINE_TRIGGER_Y).value
        self.creep_duration_sec = self.declare_parameter('creep_duration_sec', CREEP_DURATION_SEC).value
        self.path_timeout_sec = self.declare_parameter('path_timeout_sec', PATH_TIMEOUT_SEC).value
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
        self.crosswalk_latched = False   # 횡단보도 통과 중복 카운트 방지
        self.crosswalk_last_seen = None  # 마지막으로 횡단보도가 보인 시각
        self.creep_start_stamp = None

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

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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
        age = (self.now_sec() - self.path_stamp) if self.path_stamp else -1.0
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
            return self.steering_command

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

    def speed_for(self, base_speed: int, steering: int) -> int:
        """조향량이 클수록 감속."""
        ratio = abs(steering) / float(MAX_STEERING)
        return int(round(base_speed * (1.0 - self.turn_slowdown * ratio)))

    # ------------------------------------------------------------------ #
    # 랩 카운트
    # ------------------------------------------------------------------ #
    def update_lap_count(self):
        """
        횡단보도는 한 바퀴에 정확히 한 번 나타난다.
        bbox 하단이 임계선을 넘는 순간을 '통과'로 보고, latch + 쿨다운으로
        같은 통과가 여러 프레임에 걸쳐 중복 계수되는 것을 막는다.
        """
        now = self.now_sec()
        crosswalk = self.find_class(CROSSWALK_CLASS_NAME)

        if crosswalk is not None:
            self.crosswalk_last_seen = now
        elif (self.crosswalk_latched
              and self.crosswalk_last_seen is not None
              and (now - self.crosswalk_last_seen) >= self.crosswalk_clear_sec):
            # 확실히 시야에서 벗어났을 때만 래치를 푼다.
            # 단발성 미검출로 풀면 같은 횡단보도를 두 번 세게 된다.
            self.crosswalk_latched = False

        if crosswalk is None or self.bbox_bottom_y(crosswalk) < self.crosswalk_trigger_y:
            return

        if self.crosswalk_latched:
            return

        if self.last_lap_stamp is not None and (now - self.last_lap_stamp) < self.lap_cooldown_sec:
            return

        self.lap_count += 1
        self.last_lap_stamp = now
        self.crosswalk_latched = True
        self.get_logger().info(f"LAP {self.lap_count}/{self.target_lap_count} 완료")

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
            car = self.find_class(CAR_CLASS_NAME)
            car_is_near = car is not None and car.bbox.size.y >= self.car_min_bbox_height

            stopline = self.find_class(STOPLINE_CLASS_NAME)
            stopline_at_bumper = (
                stopline is not None
                and self.bbox_bottom_y(stopline) >= self.stopline_trigger_y
            )

            if car_is_near and stopline_at_bumper:
                self.get_logger().info("정지선 도달 - 차체 중앙 정렬을 위해 미소 전진")
                self.creep_start_stamp = self.now_sec()
                self.state = MissionState.CREEP

        elif self.state is MissionState.CREEP:
            # 정지선은 이미 카메라 사각(범퍼 아래)에 들어갔으므로 시간으로만 전진한다.
            elapsed = self.now_sec() - self.creep_start_stamp
            if elapsed >= self.creep_duration_sec:
                self.get_logger().info("미션 완료 - 정지")
                self.state = MissionState.STOPPED

    # ------------------------------------------------------------------ #
    # 명령 생성
    # ------------------------------------------------------------------ #
    def compute_command(self):
        # 라이다 장애물은 모든 주행 상태에 우선하는 비상 정지
        if self.obstacle_detected():
            return 0, 0, 0

        if self.state in (MissionState.WAIT_GREEN, MissionState.STOPPED):
            return 0, 0, 0

        if self.state is MissionState.CREEP:
            # 정지선을 차체 아래로 보내는 구간 - 조향 없이 직진
            return 0, self.creep_speed, self.creep_speed

        # DRIVING / APPROACH : 차선 추종
        if self.stop_on_red_while_driving and self.traffic_light_color() == 'Red':
            return 0, 0, 0

        if not self.path_is_fresh():
            # 차선을 잃은 상태에서 계속 달리면 차선 이탈로 이어진다.
            self.get_logger().warn("경로 신호 없음 - 정지", throttle_duration_sec=1.0)
            return 0, 0, 0

        steering = self.compute_steering()
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
            f"[{self.state.value}] lap: {self.lap_count}/{self.target_lap_count}, "
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
