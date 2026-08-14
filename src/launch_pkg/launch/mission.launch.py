# mission.launch.py
#
# 미션 주행용 런치 파일. (원본 main.launch.py는 수정하지 않음)
#
# main.launch.py와의 차이:
#   - lane_info_extractor_node      -> lane_extractor_node       (클래스명 'Lane2', 보정값 적용)
#   - traffic_light_detector_node   -> traffic_light_state_node  (HSV 분석 제거)
#   - motion_planner_node           -> mission_planner_node      (미션 상태 기계)
#
# 실행 인자 (기본값은 '차가 움직이지 않는' 안전한 쪽으로 설정되어 있음):
#
#   use_serial:=true      아두이노로 명령을 전송한다. 기본 false.
#                         true로 두면 /dev/ttyACM0이 없을 때 런치 전체가 즉시 죽는다.
#   data_source:=camera   실차 주행. 기본 video (번들 영상으로 벤치 테스트).
#   cam_num:=2            카메라 장치 번호. ls /dev/video* 로 확인. 기본 0.
#   device:=cpu           GPU가 준비되지 않은 경우. 기본 cuda:0.
#   show_image:=false     디버그 창을 끈다. 기본 true.
#   debug_log:=true       주행 로그를 debug_logs/*.csv 로 남긴다. 기본 false.
#   target_lap_count:=0   랩 카운트를 건너뛰고 바로 APPROACH. 정지 동작만 시험할 때.
#   params_file:=<경로>    주행/정지 튜닝 YAML. 기본값은 run.sh 가 생성하는 파일.
#
# 주의: 아두이노 포트는 serial_sender_node.py 안에 PORT 상수로 박혀 있어 인자로 바꿀 수 없다.
#       /dev/ttyACM0 이 아니면 그 파일을 직접 수정해야 한다.
#
# 벤치 테스트:  ros2 launch launch_pkg mission.launch.py
# 실차 주행:    ros2 launch launch_pkg mission.launch.py use_serial:=true data_source:=camera

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_serial = LaunchConfiguration('use_serial')
    data_source = LaunchConfiguration('data_source')
    device = LaunchConfiguration('device')
    show_image = LaunchConfiguration('show_image')
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        DeclareLaunchArgument('use_serial', default_value='false',
                              description='아두이노로 제어 명령 전송 (차량이 움직임)'),
        DeclareLaunchArgument('data_source', default_value='video',
                              description="'camera' 또는 'video'"),
        DeclareLaunchArgument('device', default_value='cuda:0',
                              description="YOLO 추론 장치 ('cuda:0' 또는 'cpu')"),
        DeclareLaunchArgument('show_image', default_value='true',
                              description='디버그 창 표시 여부'),
        DeclareLaunchArgument('threshold', default_value='0.3',
                              description='YOLO 검출 임계값'),
        DeclareLaunchArgument('wait_for_green', default_value='true',
                              description='녹색 신호 대기 후 출발 (false면 즉시 주행 - 튜닝용)'),
        DeclareLaunchArgument('debug_log', default_value='false',
                              description='주행 상태/검출 결과를 debug_logs/*.csv 로 기록'),
        DeclareLaunchArgument('target_lap_count', default_value='2',
                              description='완주 바퀴 수. 0 이면 즉시 APPROACH (정지 시험용)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(
                                  get_package_share_directory('launch_pkg'),
                                  'config', 'mission_params.yaml'),
                              description='주행/정지 튜닝 파라미터 YAML (run.sh 가 생성)'),

        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
            parameters=[{'data_source': data_source, 'logger': show_image}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
            parameters=[{'device': device, 'threshold': threshold}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_extractor_node',
            name='lane_extractor_node',
            output='screen',
            parameters=[{'show_image': show_image}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_state_node',
            name='traffic_light_state_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            # 조감도 ROI에서 차량 중심선의 위치.
            # src_mat 보정 결과, 이미지 중앙(320)은 조감도에서 x=294로 매핑된다.
            # 즉 보정 촬영 당시 차량은 차선 중심보다 약 26px(≈6cm) 왼쪽에 있었다.
            # 기본값 320을 쓰면 이 편차를 '중앙'으로 간주하여 계속 왼쪽으로 치우쳐 주행한다.
            # 카메라가 차량 중심선에 장착되어 있다는 가정이며, 실차에서 반드시 확인할 것.
            # y = 480 - cutting_idx - 1. lane_extractor_node 의 CUTTING_IDX 와 반드시 일치시킬 것.
            # (240 -> 239). 어긋나면 차량 기준점이 ROI 바닥이 아닌 곳에 놓여 경로가 왜곡된다.
            parameters=[{'car_center_point': [294, 239]}]
        ),
        Node(
            package='decision_making_pkg',
            executable='mission_planner_node',
            name='mission_planner_node',
            output='screen',
            # YAML 을 먼저 두고 런치 인자를 뒤에 둔다 -> 런치 인자가 우선한다
            parameters=[LaunchConfiguration('params_file'),
                        {'wait_for_green': LaunchConfiguration('wait_for_green'),
                         'debug_log': LaunchConfiguration('debug_log'),
                         'target_lap_count': LaunchConfiguration('target_lap_count')}]
        ),
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen',
            condition=IfCondition(use_serial)
        ),
        # 라이다는 아직 소프트웨어 검증 전이므로 비활성 상태로 둔다.
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_publisher_node',
        #     name='lidar_publisher_node',
        #     output='screen'
        # ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_processor_node',
        #     name='lidar_processor_node',
        #     output='screen'
        # ),
        # Node(
        #     package='lidar_perception_pkg',
        #     executable='lidar_obstacle_detector_node',
        #     name='lidar_obstacle_detector_node',
        #     output='screen'
        # ),
    ])
