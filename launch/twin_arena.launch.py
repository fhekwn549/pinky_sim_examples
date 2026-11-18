"""Pinky 시뮬레이션 한 번에 실행: Gazebo 서버(+선택 GUI) + 센서 브리지 + 명령 주행 + 가상 장치 + RViz.

    ros2 launch pinky_sim_examples twin_arena.launch.py                 # 기본: Gazebo 창 없이(headless), RViz로 보기
    ros2 launch pinky_sim_examples twin_arena.launch.py gz_gui:=true    # 물체 만들기 시간: Gazebo 창도 띄움
    ros2 launch pinky_sim_examples twin_arena.launch.py world_name:=my_room.world gz_world:=my_room
    ros2 launch pinky_sim_examples twin_arena.launch.py d455:=true   # RealSense D455 장착 가정(튜토리얼 예제)

공식 pinky_gz_sim/launch/launch_sim.launch.xml 은 Gazebo 창을 끄는 인자가 없어서, 그 내용을 여기로 옮겼다.
(2026-11-18 공식 repo 기준. 공식 launch가 바뀌면 아래 "공식 launch_sim 대응" 부분을 같이 확인할 것)
실행 환경별 값(~/.config/pinky_sim_examples/runtime.yaml)은 scripts/setup_env.sh 가 만든다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG_DIR = os.environ.get('PINKY_SIM_CONFIG_DIR',
                            os.path.expanduser('~/.config/pinky_sim_examples'))
RUNTIME = os.path.join(CONFIG_DIR, 'runtime.yaml')


def _layout_markers(context):
    """world_name(twin_arena.world) -> layouts/twin_arena.yaml 을 찾아 마커 노드에 넘긴다."""
    share = get_package_share_directory('pinky_sim_examples')
    stem = os.path.splitext(LaunchConfiguration('world_name').perform(context))[0]
    layout = os.path.join(share, 'layouts', f'{stem}.yaml')
    return [Node(package='pinky_sim_examples', executable='layout_markers', name='layout_markers',
                 output='screen',
                 parameters=[{'layout_file': layout if os.path.exists(layout) else '',
                              'use_sim_time': True}])]


def generate_launch_description():
    share = get_package_share_directory('pinky_sim_examples')
    gz_share = get_package_share_directory('pinky_gz_sim')
    desc_share = get_package_share_directory('pinky_description')
    ros_gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                 'gz_sim.launch.py')
    arg = LaunchConfiguration
    world = PathJoinSubstitution([share, 'worlds', arg('world_name')])

    notices = []
    driver_params = [{'use_sim_time': True}]
    if os.path.exists(RUNTIME):
        driver_params.append(RUNTIME)
        notices.append(LogInfo(msg=f'[pinky_sim_examples] 환경 설정 사용: {RUNTIME} '
                                   f'(profile={os.environ.get("PINKY_SIM_PROFILE", "?")})'))
    else:
        notices.append(LogInfo(msg='[pinky_sim_examples] 경고: 환경 설정이 없어 기본값으로 실행합니다. '
                                   'scripts/setup_env.sh 를 먼저 실행하세요'))
    if 'PINKY_SIM_PROFILE' not in os.environ and os.path.exists(RUNTIME):
        notices.append(LogInfo(msg=f'[pinky_sim_examples] 경고: env.sh 가 source 되지 않았습니다 '
                                   f'(ROS_DOMAIN_ID 등 미적용). source {CONFIG_DIR}/env.sh'))

    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'pinky_sim.urdf.xacro'),
        ' cam_tilt_deg:=8 d455:=', arg('d455')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='twin_arena.world'),
        DeclareLaunchArgument('gz_world', default_value='twin_arena',
                              description='world 파일 안의 <world name="..."> (램프 색 전달 경로)'),
        DeclareLaunchArgument('gz_gui', default_value='false',
                              description='Gazebo 창. 기본은 끔(headless) - 보기는 RViz'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('driver', default_value='true'),
        DeclareLaunchArgument('devices', default_value='true',
                              description='LED·램프·LCD 가상 장치 서버와 패널 창'),
        DeclareLaunchArgument('led_count', default_value='8'),
        DeclareLaunchArgument('d455', default_value='false',
                              description='RealSense D455 장착 가정 (docs/pinky_sim_guide.html 센서 붙이기)'),
        *notices,
        LogInfo(msg=['[pinky_sim_examples] Gazebo 창: ', arg('gz_gui'),
                     ' (false면 RViz의 회색 상자 = 레이아웃 물체, 빨간 점 = 라이다)']),

        # ---------- 공식 launch_sim 대응 ----------
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               f'{desc_share}/../:{gz_share}/models:'
                               f'{os.path.expanduser("~")}/.gazebo/models'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ros_gz_launch), launch_arguments={
            'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ros_gz_launch),
                                 launch_arguments={'gz_args': '-g -v2'}.items(),
                                 condition=IfCondition(arg('gz_gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                          'ignore_timestamp': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['joint_states'], 'rate': 20.0, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'pinky', '-topic', 'robot_description',
                        '-x', '0.0', '-y', '0.0', '-z', '0.1'],
             parameters=[{'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'sim_bridge.yaml')}]),
        Node(package='ros_gz_image', executable='image_bridge', name='image_bridge',
             arguments=['/camera/image_raw']),

        # ---------- 추가 센서 ----------
        Node(package='pinky_sim_examples', executable='us_range', name='us_range',
             parameters=[{'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='d455_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'd455_bridge.yaml')}],
             condition=IfCondition(arg('d455'))),
        LogInfo(msg='[pinky_sim_examples] 주의: d455 는 실물 Pinky 기본 구성에 없는 가정 센서입니다',
                condition=IfCondition(arg('d455'))),

        # ---------- 보기·실습 노드 ----------
        OpaqueFunction(function=_layout_markers),
        Node(package='pinky_sim_examples', executable='command_driver', name='command_driver',
             output='screen', parameters=driver_params, condition=IfCondition(arg('driver'))),
        GroupAction(condition=IfCondition(arg('devices')), actions=[
            Node(package='pinky_sim_examples', executable='sim_devices', name='sim_devices',
                 output='screen',
                 parameters=[{'gz_world': arg('gz_world'),
                              'led_count': ParameterValue(arg('led_count'), value_type=int)}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='lamp_color_bridge',
                 output='log',
                 arguments=[['/world/', arg('gz_world'), '/material_color',
                             '@ros_gz_interfaces/msg/MaterialColor]gz.msgs.MaterialColor']]),
        ]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'twin_arena.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(arg('rviz'))),
        LogInfo(msg='[pinky_sim_examples] Gazebo 창이 없습니다. 램프 색은 가상 장치 패널에서 확인하세요',
                condition=UnlessCondition(arg('gz_gui'))),
    ])
