"""
lane_extractor_node

기존 lane_info_extractor_node의 대체 노드. (원본 파일은 수정하지 않음)

원본은 차선 클래스 이름이 'lane2'로 하드코딩되어 있으나, 현재 데이터셋의 클래스 이름은
'Lane2'(대문자 L)이다. class_name 비교는 대소문자를 구분하므로 원본은 검출을 하나도
찾지 못하고, 빈 영상 -> 경로 없음 -> 조향 없음으로 아무런 오류 메시지 없이 실패한다.

변경점:
  1. 차선 클래스 이름을 파라미터로 분리 (기본값 'Lane2')
  2. 조감도 변환/ROI 관련 캘리브레이션 상수를 전부 파라미터로 분리
     (트랙에서 재빌드 없이 조정하기 위함)
  3. 빈 마스크로 인해 0x0 캔버스가 만들어지는 경우 방어

발행 토픽(yolov8_lane_info, roi_image)과 메시지 형식은 원본과 동일하다.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_lane_info"
ROI_IMAGE_TOPIC_NAME = "roi_image"

# 추종할 차선 클래스 이름 (Roboflow 라벨과 대소문자까지 정확히 일치해야 함)
# 'Lane2'는 주행 차선의 '주행 가능 영역'이므로, 그 외곽선이 곧 좌/우 경계가 된다.
# 'Lane1'을 함께 그리면 두 차선이 하나로 합쳐져 중앙선 위를 따라가게 되므로 그리지 않는다.
LANE_CLASS_NAME = 'Lane2'

# 조감도 변환 원본 좌표 (x1,y1, x2,y2, x3,y3, x4,y4)
#
# 2026-08-13 img/*.png 3장으로 실측 보정한 값.
# 지면 평면에서 차선 폭은 이미지 행 y에 대해 선형이므로(width = 1.304·y - 119, 소실점 y≈91),
# 신뢰 가능한 구간(y=230~380)에서 좌/우 경계를 직선 적합한 뒤 y=240과 y=380의 경계를 취했다.
#
# 강의 기본값 [238,316, 402,313, 501,476, 155,476]은 하단 변이 y=476으로,
# 이 차량에서는 본네트 안쪽을 가리킨다. 그 결과 ROI 스캔 15행 중 4행에서만 차선이 잡혔고
# 나머지는 조용히 lane_width/2 폴백값을 반환하고 있었다.
SRC_MAT = [258, 240, 452, 240, 547, 380, 170, 380]

# 조감도 영상에서 아래쪽만 남기기 위한 절단 위치.
# ROI 깊이 = (480 - CUTTING_IDX) 행, 조감도 1px ≈ 0.234cm (차선폭 256px = 약 60cm)
#
# 2026-08-13 실측: 조감도 각 깊이에서 차선 경계가 잡히는 비율
#   42cm(y=300) 97% · 52cm 91% · 61cm 76% · 70cm 70%
#   차선 폭은 어느 깊이에서도 260~270px 로 일정 -> 원근 보정이 먼 거리까지 유효
#
# 300 -> 240 으로 낮춰 전방 시야를 42cm -> 56cm 로 넓혔다.
# 순항속도(175)에서 전방주시 시간이 0.33s -> 0.45s 가 되어 곡선 선행이 개선된다.
# 180(70cm)까지 낮추면 스캔행의 30%가 폴백 분기로 빠지므로 여기서 멈춘다.
#
# ⚠️ 이 값을 바꾸면 launch 파일의 car_center_point y 도 (480 - CUTTING_IDX - 1) 로
#    함께 바꿔야 한다. 어긋나면 경로가 조용히 왜곡된다.
CUTTING_IDX = 240

# 경로 계획에 넘길 타겟 포인트 개수. ROI 깊이에 맞춰 자동 분배되므로
# CUTTING_IDX 를 바꿔도 따로 손댈 필요가 없다.
TARGET_POINT_COUNT = 3

# 조감도 기준 차선 폭 [px]
# dst_mat이 영상 폭의 30~70% 구간이므로 src_mat이 차선 경계에 정확히 놓이면
# 정의상 640 * 0.4 = 256px 가 된다. (보정 후 실측 257px)
LANE_WIDTH = 256

# 차선 기울기 검출 각도 제한 [deg]
THETA_LIMIT = 70

# 차선 중앙 검출 밴드 두께 [px]
DETECTION_THICKNESS = 10

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True
#----------------------------------------------


class LaneExtractorNode(Node):
    def __init__(self):
        super().__init__('lane_extractor_node')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.roi_topic = self.declare_parameter('roi_image_topic', ROI_IMAGE_TOPIC_NAME).value
        self.show_image = self.declare_parameter('show_image', SHOW_IMAGE).value

        self.lane_class_name = self.declare_parameter('lane_class_name', LANE_CLASS_NAME).value
        self.src_mat_flat = self.declare_parameter('src_mat', SRC_MAT).value
        self.cutting_idx = self.declare_parameter('cutting_idx', CUTTING_IDX).value
        self.lane_width = self.declare_parameter('lane_width', LANE_WIDTH).value
        self.theta_limit = self.declare_parameter('theta_limit', THETA_LIMIT).value
        self.detection_thickness = self.declare_parameter('detection_thickness', DETECTION_THICKNESS).value
        self.target_point_count = self.declare_parameter('target_point_count', TARGET_POINT_COUNT).value

        self.cv_bridge = CvBridge()

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(
            DetectionArray, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.publisher = self.create_publisher(LaneInfo, self.pub_topic, self.qos_profile)
        self.roi_image_publisher = self.create_publisher(Image, self.roi_topic, self.qos_profile)

        self.get_logger().info(f"lane_extractor_node started (lane class = '{self.lane_class_name}')")

    def src_mat(self):
        """파라미터로 받은 평탄화 좌표를 4점 좌표로 되돌린다."""
        f = self.src_mat_flat
        return [[f[0], f[1]], [f[2], f[3]], [f[4], f[5]], [f[6], f[7]]]

    def draw_lane_edges(self, detection_msg: DetectionArray):
        """
        지정한 클래스의 마스크 외곽선만 빈 영상에 그린다.

        원본은 CPFL.draw_edges()를 사용했으나, 그 함수는 클래스 이름이 인자로 고정되고
        캔버스 크기를 detections[0]에서 가져온다. 다중 클래스 모델에서는 0번 검출이
        차선이 아닐 수 있으므로 여기서 직접 처리한다.
        """
        height = width = 0
        for detection in detection_msg.detections:
            if detection.mask.height > 0 and detection.mask.width > 0:
                height, width = detection.mask.height, detection.mask.width
                break

        if height == 0 or width == 0:
            # 세그멘테이션 모델이 아니거나 마스크가 비어 있는 경우
            return None

        edge_image = np.zeros((height, width))
        for detection in detection_msg.detections:
            if detection.class_name == self.lane_class_name:
                edge_image = CPFL.draw_edge(edge_image, detection, color=255)
        return edge_image

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        if len(detection_msg.detections) == 0:
            return

        lane_edge_image = self.draw_lane_edges(detection_msg)
        if lane_edge_image is None:
            self.get_logger().warn(
                "마스크가 비어 있음 - 세그멘테이션 모델인지 확인할 것", throttle_duration_sec=5.0)
            return

        (h, w) = (lane_edge_image.shape[0], lane_edge_image.shape[1])  # (480, 640)
        dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)],
                   [round(w * 0.7), h], [round(w * 0.3), h]]

        lane_bird_image = CPFL.bird_convert(lane_edge_image, srcmat=self.src_mat(), dstmat=dst_mat)
        roi_image = CPFL.roi_rectangle_below(lane_bird_image, cutting_idx=self.cutting_idx)

        if self.show_image:
            cv2.imshow('lane_edge_image', lane_edge_image)
            cv2.imshow('lane_bird_img', lane_bird_image)
            cv2.imshow('roi_img', roi_image)
            cv2.waitKey(1)

        # roi_image를 uint8 형식으로 변환 (64FC1 -> uint8)
        roi_image = cv2.convertScaleAbs(roi_image)

        try:
            roi_image_msg = self.cv_bridge.cv2_to_imgmsg(roi_image, encoding="mono8")
            self.roi_image_publisher.publish(roi_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish ROI image: {e}")

        grad = CPFL.dominant_gradient(roi_image, theta_limit=self.theta_limit)

        # 타겟 포인트는 ROI 깊이에 맞춰 분배한다. cutting_idx 를 바꿔도 자동으로 따라간다.
        roi_height = roi_image.shape[0]
        step = max(1, (roi_height - 10) // self.target_point_count)
        target_ys = [5 + i * step for i in range(self.target_point_count)]

        target_xs = [
            CPFL.get_lane_center(
                roi_image,
                detection_height=y,
                detection_thickness=self.detection_thickness,
                road_gradient=grad,
                lane_width=self.lane_width)
            for y in target_ys
        ]

        # CPFL.get_lane_center 는 차선을 찾지 못해도 실패를 알리지 않고
        # lane_width/2 라는 고정값을 돌려준다. 차량 기준점(약 294)보다 한참 왼쪽이라,
        # 이 값이 그대로 경로가 되면 차량이 매 프레임 왼쪽으로 끌려간다.
        # (횡단보도 위에서 Lane2 마스크가 끊길 때 실제로 이 경로로 반대 차선에 진입했다)
        # 따라서 모든 타겟이 이 폴백값이면 '차선 못 찾음'으로 보고 발행하지 않는다.
        # -> 경로가 stale 이 되고, mission_planner 가 직진 유지로 대응한다.
        fallback = self.lane_width / 2.0
        if all(abs(x - fallback) < 1.0 for x in target_xs):
            self.get_logger().warn(
                "차선을 찾지 못함 (전부 폴백값) - LaneInfo 발행 생략",
                throttle_duration_sec=2.0)
            return

        target_points = []
        for target_point_y, target_point_x in zip(target_ys, target_xs):
            target_point = TargetPoint()
            target_point.target_x = round(target_point_x)
            target_point.target_y = round(target_point_y)
            target_points.append(target_point)

        lane = LaneInfo()
        lane.slope = grad
        lane.target_points = target_points

        self.publisher.publish(lane)


def main(args=None):
    rclpy.init(args=args)
    node = LaneExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        # Ctrl-C로 종료되는 경우 rclpy의 시그널 핸들러가 이미 context를 종료시킨 뒤이므로,
        # 여기서 다시 shutdown()을 호출하면 RCLError가 발생한다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
