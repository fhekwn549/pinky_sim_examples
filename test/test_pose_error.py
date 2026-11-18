import math

import pytest

from pinky_sim_examples.pose_error import ErrorStats, pose_diff


def test_pose_diff_distance_and_wrapped_yaw():
    pos, yaw = pose_diff((0.3, 0.4, math.radians(179)), (0.0, 0.0, math.radians(-179)))
    assert pos == pytest.approx(0.5)
    assert yaw == pytest.approx(2.0)


def test_error_stats_summary():
    st = ErrorStats()
    assert st.summary() is None
    st.add(0.02, 1.0)
    st.add(0.04, 3.0)
    assert st.summary() == '평균 3.0 cm / 최대 4.0 cm, 방향 평균 2.0° / 최대 3.0° (2회)'
