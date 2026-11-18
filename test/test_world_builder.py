import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from pinky_sim_examples.world_builder import layout_to_world, parse_layout

LAYOUTS = Path(__file__).resolve().parents[1] / 'layouts'


def test_box_center_is_half_height_and_origin_shifted():
    s = parse_layout({'origin': [1.0, 2.0],
                      'objects': [{'type': 'box', 'size': [0.2, 0.1, 0.4], 'at': [1.5, 2.0]}]})[0]
    assert (s.x, s.y, s.z) == pytest.approx((0.5, 0.0, 0.2))


def test_wall_from_to_gives_length_and_yaw():
    s = parse_layout({'objects': [{'type': 'wall', 'from': [0, 0], 'to': [0, 1]}]})[0]
    assert s.size[0] == pytest.approx(1.0)
    assert s.yaw == pytest.approx(math.pi / 2)
    assert (s.x, s.y) == pytest.approx((0.0, 0.5))


def test_duplicate_name_rejected():
    with pytest.raises(ValueError):
        parse_layout({'objects': [{'name': 'a', 'type': 'box', 'size': [1, 1, 1], 'at': [0, 0]}] * 2})


def test_dynamic_object_has_inertial():
    sdf = layout_to_world({'objects': [{'type': 'cylinder', 'radius': 0.03, 'height': 0.1,
                                        'at': [0, 0], 'static': False, 'mass': 0.1}]})
    assert '<inertial>' in sdf and '<static>false</static>' in sdf


@pytest.mark.parametrize('name', ['twin_arena.yaml', 'my_room.yaml', 'scan_room.yaml'])
def test_sample_layouts_produce_valid_xml(name):
    layout = yaml.safe_load((LAYOUTS / name).read_text(encoding='utf-8'))
    root = ET.fromstring(layout_to_world(layout).encode('utf-8'))
    world = root.find('world')
    assert world.get('name') == layout['world_name']
    assert len(world.findall('model')) == len(layout['objects']) + 1   # + ground_plane


def test_twin_arena_spawn_point_is_clear():
    """로봇 소환 위치(0,0) 반경 0.1 m 안에 물체가 없어야 한다."""
    layout = yaml.safe_load((LAYOUTS / 'twin_arena.yaml').read_text(encoding='utf-8'))
    for s in parse_layout(layout):
        r = max(s.size[:2]) / 2 if s.kind == 'box' else s.size[0]
        if s.kind == 'box' and s.size[0] > 1.0:   # 긴 벽은 축 방향 거리로 판단
            assert abs(s.y) > 0.1 or abs(s.x) > 0.1
        else:
            assert math.hypot(s.x, s.y) > r + 0.1, s.name


def test_texture_becomes_pbr_albedo_and_white_by_default():
    s = parse_layout({'objects': [{'type': 'box', 'size': [0.1, 0.1, 0.1], 'at': [0, 0],
                                   'texture': 'tex/brick.png'}]})[0]
    assert s.color == (1.0, 1.0, 1.0)
    sdf = layout_to_world({'objects': [{'type': 'box', 'size': [0.1, 0.1, 0.1], 'at': [0, 0],
                                        'texture': 'tex/brick.png', 'color': [1, 0.5, 0.5]}]})
    assert '<albedo_map>tex/brick.png</albedo_map>' in sdf and '<diffuse>1 0.5 0.5 1</diffuse>' in sdf


def test_no_texture_keeps_old_material():
    sdf = layout_to_world({'objects': [{'type': 'box', 'size': [0.1, 0.1, 0.1], 'at': [0, 0]}]})
    assert '<pbr>' not in sdf


def test_relocate_textures_relative_to_world(tmp_path):
    from pinky_sim_examples.world_builder import relocate_textures
    (tmp_path / 'layouts').mkdir()
    (tmp_path / 'textures').mkdir()
    (tmp_path / 'textures' / 'b.png').write_bytes(b'x')
    lay = {'objects': [{'name': 'a', 'type': 'box', 'size': [1, 1, 1], 'at': [0, 0],
                        'texture': '../textures/b.png'}]}
    out = relocate_textures(lay, str(tmp_path / 'layouts' / 'l.yaml'),
                            str(tmp_path / 'worlds' / 'w.world'))
    assert out['objects'][0]['texture'] == '../textures/b.png'
    stdout = relocate_textures(lay, str(tmp_path / 'layouts' / 'l.yaml'), None)
    assert stdout['objects'][0]['texture'] == str((tmp_path / 'textures' / 'b.png').resolve())
    with pytest.raises(ValueError):
        relocate_textures({'objects': [{'name': 'x', 'texture': 'nope.png'}]},
                          str(tmp_path / 'layouts' / 'l.yaml'), None)
