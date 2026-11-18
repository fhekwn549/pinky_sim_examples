"""실기 Pinky 출력 장치(LED·램프·LCD 표정)의 동작을 흉내 내는 순수 Python 모델.

ROS 의존성 없음. 규칙은 공식 repo 코드에서 그대로 옮겼다.
  LED   : pinky_led/led_server.py        (set_pixel / fill / clear, set_brightness)
  램프  : pinky_lamp_control/main_node.cpp (mode 0 끄기, 1 켜기, 2 깜빡임, 3 디밍, time[ms])
  표정  : pinky_emotion/emotion_server.py (hello, basic, angry, bored, fun, happy, interest, sad)
"""
import colorsys
from dataclasses import dataclass, field
from typing import List, Tuple

Color = Tuple[int, int, int]
OFF: Color = (0, 0, 0)
EMOTIONS = ('hello', 'basic', 'angry', 'bored', 'fun', 'happy', 'interest', 'sad')


def _clamp_rgb(r: int, g: int, b: int) -> Color:
    for v in (r, g, b):
        if not 0 <= v <= 255:
            raise ValueError(f'색 값은 0~255: {(r, g, b)}')
    return (r, g, b)


@dataclass
class LedStrip:
    """led_server.py 의 SetLed / SetBrightness 동작."""

    count: int = 8
    max_brightness: int = 100
    brightness: int = -1     # -1 = 최대. 실기 기본값은 pinkylib 내부값(비공개)
    pixels: List[Color] = field(default_factory=list)

    def __post_init__(self):
        if not self.pixels:
            self.pixels = [OFF] * self.count
        if self.brightness < 0:
            self.brightness = self.max_brightness

    def command(self, command: str, pixels: List[int], r: int, g: int, b: int) -> Tuple[bool, str]:
        """(success, message). 메시지 문구는 실기 서버와 같다."""
        command = command.lower()
        try:
            color = _clamp_rgb(r, g, b)
            if command == 'set_pixel':
                for p in pixels:
                    if not 0 <= p < self.count:
                        raise IndexError(f'pixel index {p} out of range (0~{self.count - 1})')
                for p in pixels:
                    self.pixels[p] = color
                return True, f'Set pixel(s) {list(pixels)} to color {color}.'
            if command == 'fill':
                self.pixels = [color] * self.count
                return True, f'Filled all LEDs with color {color}.'
            if command == 'clear':
                self.pixels = [OFF] * self.count
                return True, 'Cleared all LEDs.'
            return False, ("Failed: Unknown command. Available commands: "
                           "'set_pixel', 'fill', 'clear'.")
        except (IndexError, ValueError) as e:
            return False, f'Failed: {e}'

    def set_brightness(self, value: int) -> Tuple[bool, str]:
        if not 0 <= value <= self.max_brightness:
            return False, (f'Failed: Error during brightness control - '
                           f'brightness 0~{self.max_brightness}: {value}')
        self.brightness = value
        return True, f'Set LED brightness to {value}.'

    def shown(self) -> List[Color]:
        """밝기를 반영한 실제 표시 색."""
        k = self.brightness / self.max_brightness
        return [tuple(int(c * k) for c in px) for px in self.pixels]


@dataclass
class Lamp:
    """pinky_lamp_control/main_node.cpp 의 SetLamp 동작. 시작 상태는 실기와 같이 흰색 디밍 1000 ms."""

    goal: Color = (255, 255, 255)
    mode: int = 3
    time_ms: int = 1000
    start_ms: float = 0.0

    MODES = {0: '끄기', 1: '켜기', 2: '깜빡임', 3: '디밍'}

    def set(self, r: float, g: float, b: float, mode: int, time_ms: int, now_ms: float) -> Tuple[bool, str]:
        """r, g, b 는 std_msgs/ColorRGBA 처럼 0.0~1.0."""
        if mode not in self.MODES:
            return False, f'mode는 0~3: {mode}'
        if mode in (2, 3) and time_ms <= 0:
            return False, f'mode {mode}는 time(ms) > 0 이 필요합니다'
        self.goal = tuple(int(max(0.0, min(1.0, c)) * 255) for c in (r, g, b))
        self.mode, self.time_ms, self.start_ms = mode, time_ms, now_ms
        return True, (f'Set lamp to Color [{self.goal[0]} {self.goal[1]} {self.goal[2]}], '
                      f'Mode [{mode}] and Time [{time_ms}].')

    def color_at(self, now_ms: float) -> Color:
        t = now_ms - self.start_ms
        if self.mode == 0:
            return OFF
        if self.mode == 1:
            return self.goal
        if self.mode == 2:          # time_ms 동안 켜짐, time_ms 동안 꺼짐 (처음 time_ms는 꺼짐)
            return self.goal if int(t // self.time_ms) % 2 == 1 else OFF
        # mode 3: time_ms 동안 밝아지고 time_ms 동안 어두워지는 삼각파 (색상·채도 유지)
        phase = (t % (2 * self.time_ms)) / self.time_ms
        level = phase if phase <= 1.0 else 2.0 - phase
        h, s, v = colorsys.rgb_to_hsv(*(c / 255 for c in self.goal))
        return tuple(int(c * 255) for c in colorsys.hsv_to_rgb(h, s, v * level))


def check_emotion(name: str) -> Tuple[bool, str]:
    if name in EMOTIONS:
        return True, f'Emotion set to {name}'
    return False, 'Wrong command or emotion not cached'
