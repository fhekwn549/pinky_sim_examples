"""MuJoCo 로 이 패키지의 world 를 띄운다 (공식 pinky_mujoco 를 그대로 쓰고 몇 가지만 더함).

    ros2 launch pinky_sim_examples mujoco_sim.launch.py                       # nav2_practice
    ros2 launch pinky_sim_examples mujoco_sim.launch.py world_name:=twin_arena.world
    ros2 launch pinky_sim_examples mujoco_sim.launch.py viewer:=true          # MuJoCo 창 보기
    ros2 launch pinky_sim_examples mujoco_sim.launch.py physics_config:=$HOME/my_mujoco.yaml

공식 launch(pinky_mujoco/launch_sim.launch.py)와 다른 점
  - world_name 을 이 패키지 worlds/ 에서 먼저 찾는다(없으면 공식 pinky_gz_sim/worlds, pinky_mujoco/worlds)
  - physics_config: 물리 설정 YAML 을 통째로 바꿔 끼운다(공식 config/mujoco.yaml 을 복사해서 고칠 것)
  - command_driver(문장 명령 주행)와 RViz 를 함께 띄운다. 기본은 MuJoCo 창 없이(viewer:=false)
나머지(로봇 모델, 토픽, /ground_truth)는 공식과 같다.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_DIR = os.environ.get('PINKY_SIM_CONFIG_DIR', os.path.expanduser('~/.config/pinky_sim_examples'))
RUNTIME = os.path.join(CONFIG_DIR, 'runtime.yaml')


def find_world(name: str) -> Path:
    for pkg, sub in (('pinky_sim_examples', 'worlds'), ('pinky_gz_sim', 'worlds'), ('pinky_mujoco', 'worlds')):
        try:
            path = Path(get_package_share_directory(pkg)) / sub / name
        except Exception:
            continue
        if path.is_file():
            return path
    raise FileNotFoundError(f'world 를 찾지 못했습니다: {name}')


def setup(context):
    from pinky_mujoco.description import expand_urdf

    arg = lambda k: LaunchConfiguration(k).perform(context)
    flag = lambda k: arg(k).lower() == 'true'
    urdf = expand_urdf(mappings={'cam_tilt_deg': arg('cam_tilt_deg'), 'screen_tilt_deg': '-25'})
    model_dir = Path(os.environ.get('PINKY_MODEL_DIR', str(Path.home() / '.cache/pinky_mujoco')))
    model_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = model_dir / 'robot.urdf'
    urdf_path.write_text(urdf)
    world = Path(arg('world')).expanduser() if arg('world') else find_world(arg('world_name'))

    bridge_args = ['--urdf-file', str(urdf_path), '--output-dir', str(model_dir), '--world', str(world)]
    env = {}
    if flag('viewer'):
        bridge_args.append('--viewer')
    if not flag('camera'):
        bridge_args.append('--no-camera')
    if not flag('use_gpu'):
        bridge_args.append('--simple-render')
        env = {'MUJOCO_GL': 'glfw', 'LIBGL_ALWAYS_SOFTWARE': '1', '__GLX_VENDOR_LIBRARY_NAME': 'mesa'}
    if arg('physics_config'):
        bridge_args += ['--physics-config', str(Path(arg('physics_config')).expanduser())]

    driver_params = [{'use_sim_time': True}] + ([RUNTIME] if os.path.exists(RUNTIME) else [])
    share = get_package_share_directory('pinky_sim_examples')
    actions = [
        LogInfo(msg=f'[pinky_sim_examples] MuJoCo world={world} physics_config={arg("physics_config") or "(공식 기본값)"}'),
        Node(package='pinky_mujoco', executable='bridge', name='pinky_mujoco', arguments=bridge_args,
             output='screen', additional_env=env, parameters=[{'use_sim_time': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='log', parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
    ]
    if flag('driver'):
        actions.append(Node(package='pinky_sim_examples', executable='command_driver', name='command_driver',
                            output='screen', parameters=driver_params))
    if flag('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(share, 'rviz', 'twin_arena.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='nav2_practice.world',
                              description='이 패키지 worlds/ -> pinky_gz_sim/worlds -> pinky_mujoco/worlds 순서로 찾음'),
        DeclareLaunchArgument('world', default_value='', description='world 절대 경로(world_name 보다 우선)'),
        DeclareLaunchArgument('physics_config', default_value='', description='물리 설정 YAML(통째로 교체)'),
        DeclareLaunchArgument('viewer', default_value='false', description='MuJoCo 창'),
        DeclareLaunchArgument('camera', default_value='true', description='/camera/image_raw 발행'),
        DeclareLaunchArgument('use_gpu', default_value='false', description='true: GPU 로 그리기(드라이버 필요)'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8'),
        DeclareLaunchArgument('driver', default_value='true', description='command_driver(Nav2 를 쓸 땐 false)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
