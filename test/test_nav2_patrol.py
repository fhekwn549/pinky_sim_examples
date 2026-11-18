"""nav2_patrol 인자 처리 (ROS 없이 확인: rclpy import 전에 끝나는 경로만)."""
import pytest

from pinky_sim_examples import nav2_patrol


@pytest.mark.parametrize('argv', [['G9'], ['G2', 'X1']])
def test_unknown_goal_rejected(argv):
    with pytest.raises(SystemExit) as e:
        nav2_patrol.main(argv)
    assert e.value.code == 2


def test_goal_table_matches_layout_comment():
    assert nav2_patrol.GOALS['G1'] == (0.0, 0.0, 0.0)
    assert set(nav2_patrol.GOALS) == {'G1', 'G2', 'G3', 'G4', 'G5'}
