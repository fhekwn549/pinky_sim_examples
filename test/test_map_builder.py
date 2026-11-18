"""make_map: 레이아웃 -> Nav2 지도 변환 확인."""
import math
from pathlib import Path

import pytest
import yaml

from pinky_sim_examples.map_builder import FREE, OCCUPIED, UNKNOWN, build_grid, write_map
from pinky_sim_examples.world_builder import parse_layout

LAYOUT = Path(__file__).resolve().parents[1] / 'layouts' / 'nav2_practice.yaml'


def cell(rows, origin, x, y, res=0.05):
    return rows[int((y - origin[1]) / res)][int((x - origin[0]) / res)]


@pytest.fixture(scope='module')
def practice():
    layout = yaml.safe_load(LAYOUT.read_text(encoding='utf-8'))
    rows, origin = build_grid(layout)
    return layout, rows, origin


def test_walls_occupied_and_start_free(practice):
    _, rows, origin = practice
    assert cell(rows, origin, 0.0, 0.0) == FREE
    assert cell(rows, origin, 1.0, -0.6) == OCCUPIED      # 얇은 벽(0.03 m)도 빠지지 않음
    assert cell(rows, origin, 0.0, 0.7) == OCCUPIED       # 가운데 벽


def test_door_and_narrow_gap_are_free(practice):
    _, rows, origin = practice
    assert cell(rows, origin, 0.8, 0.7) == FREE           # 문
    assert cell(rows, origin, 2.1, 0.7) == FREE           # 좁은 틈(정적 지도에서는 빈칸)


def test_low_step_not_on_map_but_tall_objects_are(practice):
    _, rows, origin = practice
    assert cell(rows, origin, 0.55, -0.30) == FREE        # 0.08 m 턱: 라이다보다 낮음
    assert cell(rows, origin, 1.0, 0.05) == OCCUPIED      # 기둥


def test_outside_walls_unknown(practice):
    _, rows, origin = practice
    assert rows[0][0] == UNKNOWN


@pytest.mark.parametrize('goal', [(0.0, 0.0), (-0.1, 1.4), (2.2, 1.1), (2.3, -0.3), (0.55, -0.3)])
def test_goals_are_free_cells(practice, goal):
    _, rows, origin = practice
    assert cell(rows, origin, *goal) == FREE


def test_goals_clear_of_tall_objects(practice):
    """G1~G4 는 로봇 반폭(0.06)+여유 0.1 m 안에 높은 물체가 없어야 한다."""
    layout, _, _ = practice
    from pinky_sim_examples.map_builder import _covers, _height
    tall = [s for s in parse_layout(layout) if _height(s) > 0.125]
    for gx, gy in [(0.0, 0.0), (-0.1, 1.4), (2.2, 1.1), (2.3, -0.3)]:
        for a in range(0, 360, 30):
            px, py = gx + 0.16 * math.cos(math.radians(a)), gy + 0.16 * math.sin(math.radians(a))
            assert not any(_covers(s, px, py, 0.0) for s in tall), (gx, gy)


def test_write_map_format(tmp_path, practice):
    _, rows, origin = practice
    write_map(rows, origin, 0.05, str(tmp_path / 'm'))
    meta = yaml.safe_load((tmp_path / 'm.yaml').read_text())
    assert meta['image'] == 'm.pgm' and meta['mode'] == 'trinary' and meta['resolution'] == 0.05
    data = (tmp_path / 'm.pgm').read_bytes()
    assert data.startswith(b'P5')
    header_end = data.index(b'255\n') + 4
    assert len(data) - header_end == len(rows) * len(rows[0])


def test_start_inside_object_rejected():
    with pytest.raises(ValueError):
        build_grid({'objects': [{'type': 'box', 'size': [1, 1, 1], 'at': [0, 0]}]})
