"""
traffic_light_state_node

기존 traffic_light_detector_node의 대체 노드. (원본 파일은 수정하지 않음)

원본은 'traffic_light' 클래스의 bbox를 잘라내어 HSV 픽셀 비율로 점등 색을 판정했으나,
현재 데이터셋은 신호등 하우징 자체를 점등 상태별로 라벨링하고 있다.
  - 'Red_light'   : 적색 점등 상태의 하우징
  - 'Greenlight'  : 녹색 점등 상태의 하우징
따라서 색상 분석이 불필요하며, 클래스 이름만으로 상태를 판정한다.

이에 따라 image_raw 구독, CvBridge, ApproximateTimeSynchronizer가 모두 제거되었다.
발행 토픽과 메시지 형식('Red'/'Green'/'None')은 원본과 동일하므로
motion_planner 계열 노드는 수정 없이 그대로 연결된다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import DetectionArray
from std_msgs.msg import String

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_traffic_light_info"

# 신호등 클래스 이름 (Roboflow 라벨과 대소문자까지 정확히 일치해야 함)
RED_CLASS_NAME = 'Red_light'
GREEN_CLASS_NAME = 'Greenlight'

# 이 점수 미만의 검출은 무시
MIN_SCORE = 0.3
# ----------------------------------------------


class TrafficLightStateNode(Node):
    def __init__(self):
        super().__init__('traffic_light_state_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.min_score = self.declare_parameter('min_score', MIN_SCORE).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(
            DetectionArray, self.sub_detection_topic, self.detections_callback, self.qos_profile)

        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

        self.get_logger().info(
            f"traffic_light_state_node started (red='{RED_CLASS_NAME}', green='{GREEN_CLASS_NAME}')")

    def detections_callback(self, detection_msg: DetectionArray):
        # 신호등이 여러 개 검출되면 score가 가장 높은 것을 신뢰한다.
        # (적색/녹색이 한 프레임에 동시 검출되어도 하나의 상태만 발행됨)
        traffic_light_color = 'None'
        best_score = self.min_score

        for detection in detection_msg.detections:
            if detection.class_name == RED_CLASS_NAME:
                color = 'Red'
            elif detection.class_name == GREEN_CLASS_NAME:
                color = 'Green'
            else:
                continue

            if detection.score >= best_score:
                traffic_light_color = color
                best_score = detection.score

        color_msg = String()
        color_msg.data = traffic_light_color
        self.publisher.publish(color_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        # Ctrl-C로 종료되는 경우 rclpy의 시그널 핸들러가 이미 context를 종료시킨 뒤이므로,
        # 여기서 다시 shutdown()을 호출하면 RCLError가 발생한다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
