"""Gazebo가 켜져 있는 동안 물체를 추가/삭제한다. world 파일은 바꾸지 않는다.

  ros2 run pinky_sim_examples spawn_object box --size 0.1 0.1 0.2 --at 0.4 0 --name box1
  ros2 run pinky_sim_examples spawn_object cylinder --radius 0.04 --height 0.2 --at 0.3 0.3
  ros2 run pinky_sim_examples spawn_object wall --from 0.5 -0.3 --to 0.5 0.3 --name gate
  ros2 run pinky_sim_examples spawn_object remove --name box1
  ros2 run pinky_sim_examples spawn_object box --at 0.5 0 --texture ~/textures/brick.png --name brick

좌표는 Gazebo world 좌표(로봇 소환 위치 = 0, 0). 내부적으로 world_builder로 SDF 문자열을 만들고
`ros2 run ros_gz_sim create -string ...` 를 호출한다.
"""
import argparse
import os
import subprocess
import sys

from pinky_sim_examples.world_builder import parse_layout, shape_to_sdf


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='실행 중인 Gazebo에 물체 추가/삭제')
    p.add_argument('type', choices=['box', 'cylinder', 'wall', 'remove'])
    p.add_argument('--name', default=None)
    p.add_argument('--world', default='twin_arena', help='world 이름(<world name="...">)')
    p.add_argument('--at', nargs=2, type=float, metavar=('X', 'Y'))
    p.add_argument('--size', nargs=3, type=float, metavar=('SX', 'SY', 'SZ'))
    p.add_argument('--radius', type=float)
    p.add_argument('--height', type=float, default=0.25)
    p.add_argument('--from', dest='p_from', nargs=2, type=float, metavar=('X', 'Y'))
    p.add_argument('--to', dest='p_to', nargs=2, type=float, metavar=('X', 'Y'))
    p.add_argument('--thickness', type=float, default=0.02)
    p.add_argument('--yaw', type=float, default=0.0, help='도(degree)')
    p.add_argument('--color', nargs=3, type=float, default=None,
                   help='r g b (0~1). 기본: 주황, --texture 가 있으면 흰색(텍스처 원래 색)')
    p.add_argument('--texture', default=None,
                   help='입힐 이미지(png/jpg). 절대 경로로 바꿔 넘긴다')
    p.add_argument('--dynamic', action='store_true', help='밀리는 물체(static false)')
    p.add_argument('--mass', type=float, default=0.2)
    p.add_argument('--dry-run', action='store_true', help='SDF만 출력하고 소환하지 않음')
    return p


def to_object(a) -> dict:
    color = a.color or ([1.0, 1.0, 1.0] if a.texture else [0.9, 0.45, 0.35])
    obj = {'type': a.type, 'name': a.name or f'{a.type}_spawned', 'color': color,
           'static': not a.dynamic, 'mass': a.mass, 'yaw': a.yaw}
    if a.texture:
        # -string 으로 넘긴 SDF 는 기준 파일 위치가 없어서 상대 경로를 못 찾는다 -> 절대 경로
        tex = os.path.abspath(a.texture)
        if not os.path.exists(tex):
            raise SystemExit(f'텍스처 파일이 없습니다: {tex}')
        obj['texture'] = tex
    if a.type == 'box':
        obj.update(size=a.size or [0.1, 0.1, a.height], at=a.at or [0.5, 0.0])
    elif a.type == 'cylinder':
        obj.update(radius=a.radius or 0.04, height=a.height, at=a.at or [0.5, 0.0])
    elif a.type == 'wall':
        if not (a.p_from and a.p_to):
            raise SystemExit('wall 은 --from X Y --to X Y 가 필요합니다')
        obj.update({'from': a.p_from, 'to': a.p_to, 'thickness': a.thickness, 'height': a.height})
    return obj


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    if a.type == 'remove':
        if not a.name:
            raise SystemExit('remove 는 --name 이 필요합니다')
        cmd = ['gz', 'service', '-s', f'/world/{a.world}/remove',
               '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', f'name: "{a.name}" type: MODEL']
        rc = subprocess.call(cmd)
        if rc == 0:
            notify_markers({'op': 'remove', 'name': a.name})
        return rc

    obj = to_object(a)
    shape = parse_layout({'objects': [obj]})[0]
    sdf = f'<sdf version="1.8">\n{shape_to_sdf(shape, indent="")}\n</sdf>'
    if a.dry_run:
        print(sdf)
        return 0
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', a.world, '-name', shape.name,
           '-string', sdf, '-x', str(shape.x), '-y', str(shape.y), '-z', str(shape.z),
           '-Y', str(shape.yaw)]
    rc = subprocess.call(cmd)
    if rc == 0:
        notify_markers({'op': 'add', 'object': obj})
    return rc


def notify_markers(event: dict, timeout: float = 3.0):
    """layout_markers 노드에 추가·삭제를 알린다(RViz 마커 갱신). 노드가 없으면 조용히 넘어간다."""
    import json
    import time

    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('spawn_object_notify')
    pub = node.create_publisher(String, 'sim_objects', 10)
    deadline = time.monotonic() + timeout
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count():
        pub.publish(String(data=json.dumps(event)))
        time.sleep(0.5)
    else:
        print('layout_markers 노드가 없어 RViz 마커는 갱신하지 않았습니다')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
