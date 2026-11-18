"""레이아웃(YAML) -> Gazebo SDF world 변환기.

ROS 의존성이 없는 순수 Python 모듈이다. "바닥 중심 좌표 + 크기"만 적으면
SDF <model>을 만들어 준다(높이 중심 계산, 벽의 길이·각도 계산은 여기서 한다).

지원하는 물체 type
  box      : size [X폭, Y깊이, 높이], at [x, y] (바닥 중심), yaw(도)
  cylinder : radius, height, at [x, y]
  wall     : from [x, y], to [x, y], thickness, height  (두 점을 잇는 벽)

공통 옵션: name, color [r, g, b], static(기본 true), mass(static: false일 때)
         texture: 이미지 경로(png/jpg). 레이아웃 파일 위치 기준 상대 경로 또는 절대 경로.
                  make_world 가 출력 world 파일 위치 기준 상대 경로로 바꿔 적는다.
                  color 를 함께 주면 텍스처에 그 색이 곱해진다(기본 흰색 = 원래 색).
"""
import os
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import yaml

DEFAULT_COLOR = (0.55, 0.63, 0.67)


@dataclass(frozen=True)
class Shape:
    name: str
    kind: str                 # 'box' | 'cylinder'
    x: float                  # Gazebo world 좌표(원점 보정 후), 물체 중심
    y: float
    z: float
    yaw: float                # 라디안
    size: Sequence[float]     # box: (sx, sy, sz) / cylinder: (radius, height)
    color: Sequence[float]
    static: bool = True
    mass: float = 1.0
    texture: Optional[str] = None   # SDF 에 그대로 적을 경로 (world 파일 기준 상대 또는 절대)


def _wall_to_box(obj: dict, height: float) -> dict:
    (x1, y1), (x2, y2) = obj['from'], obj['to']
    length = math.hypot(x2 - x1, y2 - y1)
    if length <= 0:
        raise ValueError(f"wall '{obj.get('name')}': from과 to가 같습니다")
    return {
        **obj,
        'type': 'box',
        'size': [length, obj.get('thickness', 0.02), obj.get('height', height)],
        'at': [(x1 + x2) / 2, (y1 + y2) / 2],
        'yaw': math.degrees(math.atan2(y2 - y1, x2 - x1)),
    }


def parse_layout(layout: dict) -> List[Shape]:
    """레이아웃 dict를 Shape 목록으로 바꾼다. origin 점이 Gazebo (0, 0)이 된다."""
    ox, oy = layout.get('origin', [0.0, 0.0])
    wall_h = layout.get('wall_height', 0.25)
    shapes: List[Shape] = []
    names = set()
    for i, raw in enumerate(layout.get('objects', [])):
        obj = _wall_to_box(raw, wall_h) if raw.get('type') == 'wall' else raw
        kind = obj.get('type')
        name = obj.get('name', f'{kind}_{i}')
        if name in names:
            raise ValueError(f"물체 이름 중복: {name}")
        names.add(name)
        x, y = obj['at'][0] - ox, obj['at'][1] - oy
        if kind == 'box':
            size = tuple(obj['size'])
            if len(size) != 3:
                raise ValueError(f"{name}: box size는 [X폭, Y깊이, 높이] 3개")
            z = size[2] / 2
        elif kind == 'cylinder':
            size = (obj['radius'], obj['height'])
            z = size[1] / 2
        else:
            raise ValueError(f"{name}: 알 수 없는 type '{kind}'")
        texture = obj.get('texture')
        default_color = (1.0, 1.0, 1.0) if texture else DEFAULT_COLOR
        shapes.append(Shape(
            name=name, kind=kind, x=x, y=y, z=z,
            yaw=math.radians(obj.get('yaw', 0.0)), size=size,
            color=tuple(obj.get('color', default_color)),
            static=obj.get('static', True), mass=obj.get('mass', 1.0),
            texture=texture,
        ))
    return shapes


def relocate_textures(layout: dict, layout_path: str, world_path: Optional[str]) -> dict:
    """texture 경로를 '레이아웃 파일 기준'에서 'world 파일 기준'으로 바꾼 사본을 돌려준다.

    world_path 가 없으면(화면 출력) 절대 경로로 바꾼다. 이미 절대 경로면 그대로 둔다.
    """
    base = os.path.dirname(os.path.abspath(layout_path))
    out_dir = os.path.dirname(os.path.abspath(world_path)) if world_path else None
    objects = []
    for obj in layout.get('objects', []):
        tex = obj.get('texture')
        if tex and not os.path.isabs(tex):
            full = os.path.normpath(os.path.join(base, tex))
            if not os.path.exists(full):
                raise ValueError(f"{obj.get('name')}: 텍스처 파일이 없습니다: {full}")
            obj = {**obj, 'texture': os.path.relpath(full, out_dir).replace(os.sep, '/')
                   if out_dir else full}
        objects.append(obj)
    return {**layout, 'objects': objects}


def _geometry(s: Shape) -> str:
    if s.kind == 'box':
        return f"<box><size>{s.size[0]:.4f} {s.size[1]:.4f} {s.size[2]:.4f}</size></box>"
    return (f"<cylinder><radius>{s.size[0]:.4f}</radius>"
            f"<length>{s.size[1]:.4f}</length></cylinder>")


def _inertia(s: Shape) -> str:
    m = s.mass
    if s.kind == 'box':
        a, b, c = s.size
        ixx, iyy, izz = m * (b*b + c*c) / 12, m * (a*a + c*c) / 12, m * (a*a + b*b) / 12
    else:
        r, h = s.size
        ixx = iyy = m * (3*r*r + h*h) / 12
        izz = m * r * r / 2
    return (f"<inertial><mass>{m}</mass><inertia>"
            f"<ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>"
            f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")


def shape_to_sdf(s: Shape, indent: str = '    ') -> str:
    """Shape 하나를 SDF <model>로. ros_gz_sim create -string 에도 그대로 쓸 수 있다."""
    r, g, b = s.color
    geom = _geometry(s)
    inertial = '' if s.static else _inertia(s)
    # 텍스처: Gazebo(Harmonic)는 PBR albedo_map 으로 입힌다. diffuse 색이 텍스처에 곱해진다
    pbr = (f'<pbr><metal><albedo_map>{s.texture}</albedo_map></metal></pbr>'
           if s.texture else '')
    return '\n'.join([
        f'{indent}<model name="{s.name}">',
        f'{indent}  <static>{"true" if s.static else "false"}</static>',
        f'{indent}  <pose>{s.x:.4f} {s.y:.4f} {s.z:.4f} 0 0 {s.yaw:.4f}</pose>',
        f'{indent}  <link name="link">',
    ] + ([f'{indent}    {inertial}'] if inertial else []) + [
        f'{indent}    <collision name="collision"><geometry>{geom}</geometry></collision>',
        f'{indent}    <visual name="visual"><geometry>{geom}</geometry>',
        f'{indent}      <material><ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse>{pbr}</material>',
        f'{indent}    </visual>',
        f'{indent}  </link>',
        f'{indent}</model>',
    ])


WORLD_HEADER = """<?xml version="1.0"?>
<!-- 자동 생성 파일: make_world {source} 로 다시 만드세요. 직접 고치면 덮어써집니다. -->
<sdf version="1.8">
  <world name="{world}">
    <!-- pinky_gz_sim/worlds/empty.world 와 같은 시스템 플러그인. Sensors가 없으면 /scan이 안 나온다 -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

WORLD_FOOTER = """  </world>
</sdf>
"""


def layout_to_world(layout: dict, source: str = 'layout.yaml') -> str:
    shapes = parse_layout(layout)
    body = '\n'.join(shape_to_sdf(s) for s in shapes)
    world = layout.get('world_name', 'default')
    return WORLD_HEADER.format(world=world, source=source) + body + '\n' + WORLD_FOOTER


def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    p = argparse.ArgumentParser(description='레이아웃 YAML -> Gazebo world(SDF)')
    p.add_argument('layout', help='입력 레이아웃 YAML')
    p.add_argument('-o', '--output', help='출력 .world 경로(생략 시 화면 출력)')
    a = p.parse_args(argv)
    with open(a.layout, encoding='utf-8') as f:
        layout = yaml.safe_load(f)
    layout = relocate_textures(layout, a.layout, a.output)
    sdf = layout_to_world(layout, source=a.layout)
    if a.output:
        with open(a.output, 'w', encoding='utf-8') as f:
            f.write(sdf)
        print(f'{a.output}: 물체 {len(parse_layout(layout))}개')
    else:
        print(sdf)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
