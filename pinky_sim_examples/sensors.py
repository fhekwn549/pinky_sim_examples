"""가상 센서 값 변환 (ROS 의존 없음, 단위 테스트 대상)."""
import math
from typing import Sequence

# 실기 pinky_sensor_adc: range = 1.0 * (adc / 4096.0) - 0.03  ->  adc 최대(4096)일 때 0.97 m
REAL_US_CEILING = 1.0 - 0.03


def us_range(ranges: Sequence[float], min_range: float = 0.02,
             ceiling: float = REAL_US_CEILING) -> float:
    """좁은 라이다 빔들 -> 초음파 한 값. 가장 가까운 유효값, 없거나 멀면 ceiling."""
    best = math.inf
    for r in ranges:
        if math.isfinite(r) and r >= min_range:
            best = min(best, r)
    return min(best, ceiling)
