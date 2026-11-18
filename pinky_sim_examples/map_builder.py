"""레이아웃(YAML) -> Nav2 지도(pgm + yaml) 변환기. 순수 Python(ROS·numpy 의존 없음).

SLAM으로 돌아다니며 만들지 않고, 레이아웃에서 바로 "정답 지도"를 계산한다.
  - 라이다 높이(기본 0.125 m)보다 높은 물체만 점유(검정)로 칠한다. SLAM 지도와 같은 기준이다.
  - 로봇 출발점(0, 0)에서 이어진 빈 곳만 빈칸(흰색), 벽 바깥처럼 닿을 수 없는 곳은 모름(회색).
  - 좌표는 Gazebo world 좌표 그대로다. 공식 AMCL 초기 위치(0, 0, 0)와 바로 맞는다.

  ros2 run pinky_sim_examples make_map layouts/nav2_practice.yaml -o maps/nav2_practice
  -> maps/nav2_practice.pgm, maps/nav2_practice.yaml (map_saver_cli 와 같은 형식)
"""
import math
import os
from collections import deque
from typing import List, Optional, Tuple

import yaml

from pinky_sim_examples.world_builder import Shape, parse_layout

OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def _height(s: Shape) -> float:
    return s.size[2] if s.kind == 'box' else s.size[1]


def _covers(s: Shape, x: float, y: float, pad: float) -> bool:
    """칸 중심 (x, y)가 물체 바닥면(+pad)에 들어가나. pad 로 얇은 벽이 칸 사이로 빠지지 않게 한다."""
    dx, dy = x - s.x, y - s.y
    if s.kind == 'cylinder':
        return math.hypot(dx, dy) <= s.size[0] + pad
    c, sn = math.cos(-s.yaw), math.sin(-s.yaw)
    lx, ly = dx * c - dy * sn, dx * sn + dy * c
    return abs(lx) <= s.size[0] / 2 + pad and abs(ly) <= s.size[1] / 2 + pad


def _footprint_bounds(s: Shape) -> Tuple[float, float, float, float]:
    if s.kind == 'cylinder':
        r = s.size[0]
        return s.x - r, s.y - r, s.x + r, s.y + r
    hx, hy = s.size[0] / 2, s.size[1] / 2
    c, sn = abs(math.cos(s.yaw)), abs(math.sin(s.yaw))
    ex, ey = hx * c + hy * sn, hx * sn + hy * c
    return s.x - ex, s.y - ey, s.x + ex, s.y + ey


def build_grid(layout: dict, resolution: float = 0.05, lidar_height: float = 0.125,
               margin: float = 0.2, start: Tuple[float, float] = (0.0, 0.0)):
    """(rows, origin_xy) 를 돌려준다. rows[0] 이 지도 맨 아래(y 최소) 줄이다."""
    shapes = parse_layout(layout)
    seen = [s for s in shapes if _height(s) > lidar_height]
    if not shapes:
        raise ValueError('물체가 없습니다')
    b = [_footprint_bounds(s) for s in shapes]
    x0 = min(v[0] for v in b) - margin
    y0 = min(v[1] for v in b) - margin
    w = int(math.ceil((max(v[2] for v in b) + margin - x0) / resolution))
    h = int(math.ceil((max(v[3] for v in b) + margin - y0) / resolution))
    pad = resolution / 2
    occ = [[any(_covers(s, x0 + (i + 0.5) * resolution, y0 + (j + 0.5) * resolution, pad)
                for s in seen) for i in range(w)] for j in range(h)]

    # 출발점에서 4방향으로 이어진 빈칸만 FREE
    rows = [[OCCUPIED if occ[j][i] else UNKNOWN for i in range(w)] for j in range(h)]
    si, sj = int((start[0] - x0) / resolution), int((start[1] - y0) / resolution)
    if not (0 <= si < w and 0 <= sj < h) or occ[sj][si]:
        raise ValueError(f'출발점 {start} 이 지도 밖이거나 물체 안에 있습니다')
    q = deque([(si, sj)])
    rows[sj][si] = FREE
    while q:
        i, j = q.popleft()
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= ni < w and 0 <= nj < h and rows[nj][ni] == UNKNOWN:
                rows[nj][ni] = FREE
                q.append((ni, nj))
    return rows, (x0, y0)


def write_map(rows: List[List[int]], origin: Tuple[float, float], resolution: float, out: str):
    """out(확장자 없이) -> out.pgm, out.yaml. map_saver_cli 와 같은 trinary 형식."""
    h, w = len(rows), len(rows[0])
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out + '.pgm', 'wb') as f:
        f.write(f'P5\n# pinky_sim_examples make_map\n{w} {h}\n255\n'.encode())
        for row in reversed(rows):             # pgm 첫 줄 = 지도 맨 위(y 최대)
            f.write(bytes(row))
    meta = {
        'image': os.path.basename(out) + '.pgm',
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [round(origin[0], 4), round(origin[1], 4), 0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(out + '.yaml', 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    p = argparse.ArgumentParser(description='레이아웃 YAML -> Nav2 지도(pgm + yaml)')
    p.add_argument('layout')
    p.add_argument('-o', '--output', required=True, help='출력 경로(확장자 없이). 예: maps/nav2_practice')
    p.add_argument('--resolution', type=float, default=0.05)
    p.add_argument('--lidar-height', type=float, default=0.125,
                   help='이보다 낮은 물체는 지도에 넣지 않는다(라이다가 못 봄)')
    a = p.parse_args(argv)
    with open(a.layout, encoding='utf-8') as f:
        layout = yaml.safe_load(f)
    rows, origin = build_grid(layout, a.resolution, a.lidar_height)
    write_map(rows, origin, a.resolution, a.output)
    flat = [v for r in rows for v in r]
    print(f'{a.output}.pgm/.yaml: {len(rows[0])} x {len(rows)} 칸, 점유 {flat.count(OCCUPIED)}, '
          f'빈칸 {flat.count(FREE)}, 모름 {flat.count(UNKNOWN)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
