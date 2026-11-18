import math

import pytest

from pinky_sim_examples.commands import Step, parse, sector_min, wrap_angle


def test_forward_and_turn_sequence():
    assert parse('forward 0.5; turn 90') == [Step('move', 0.5), Step('turn', math.pi / 2)]


def test_back_is_negative_move():
    assert parse('BACK 0.2') == [Step('move', -0.2)]


def test_square_expands_to_eight_steps():
    steps = parse('square 0.3')
    assert len(steps) == 8
    assert steps[0] == Step('move', 0.3) and steps[1].kind == 'turn'


def test_stop_and_speed():
    assert parse('stop') == [Step('stop', 0.0)]
    assert parse('speed 0.05') == [Step('speed', 0.05)]


@pytest.mark.parametrize('bad', ['', 'jump 1', 'forward', 'forward x', 'forward -1',
                                 'turn 0', 'speed 2', 'forward 0.1; fly 2'])
def test_invalid_commands_raise(bad):
    with pytest.raises(ValueError):
        parse(bad)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_sector_min_respects_mount_yaw():
    # 라이다 4방향: 0(라이다 앞), 90, 180, 270도. 라이다가 pi 돌아가 장착 -> 라이다 180도 = 로봇 앞
    ranges = [1.0, 2.0, 0.3, 4.0]
    inc = math.pi / 2
    front = sector_min(ranges, 0.0, inc, center=0.0, half_width=0.1, mount_yaw=math.pi)
    rear = sector_min(ranges, 0.0, inc, center=math.pi, half_width=0.1, mount_yaw=math.pi)
    assert front == 0.3 and rear == 1.0


def test_sector_min_ignores_invalid():
    assert sector_min([float('inf'), 0.01], 0.0, 0.1, 0.0, 1.0, range_min=0.05) == math.inf


def test_rate_hz():
    from pinky_sim_examples.commands import rate_hz
    assert rate_hz([0.0, 0.1, 0.2, 0.3]) == pytest.approx(10.0)
    assert rate_hz([1.0]) == 0.0 and rate_hz([1.0, 1.0]) == 0.0
