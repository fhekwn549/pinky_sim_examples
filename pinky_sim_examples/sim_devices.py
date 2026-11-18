"""시뮬레이션용 Pinky 출력 장치 서버 + 확인용 패널 창.

실기와 같은 서비스 이름·타입을 제공한다. 실기 코드에서 부르던 서비스 호출을 그대로 보내면 된다.
  /set_led         pinky_interfaces/srv/SetLed         -> 패널의 LED 줄
  /set_brightness  pinky_interfaces/srv/SetBrightness  -> 패널의 LED 밝기
  /set_lamp        pinky_interfaces/srv/SetLamp        -> 패널 램프 + Gazebo 로봇 램프 색
  /set_emotion     pinky_interfaces/srv/Emotion        -> 패널의 LCD(표정 GIF)

Gazebo 램프 색은 /world/<gz_world>/material_color (ros_gz_interfaces/MaterialColor)로 보낸다.
launch 의 ros_gz_bridge 가 이것을 Gazebo로 넘긴다.
"""
import os
import threading
import time
from collections import deque

import rclpy
from pinky_interfaces.srv import Emotion, SetBrightness, SetLamp, SetLed
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, MaterialColor
from std_msgs.msg import ColorRGBA

from pinky_sim_examples.devices import Lamp, LedStrip, check_emotion


# 패널이 찾는 한글 폰트 (앞에서부터). 맑은 고딕은 WSL에서 setup_env.sh --windows-fonts 로 연결
KOREAN_FONTS = ('Malgun Gothic', '맑은 고딕', 'Noto Sans CJK KR', 'Noto Sans KR',
                'NanumGothic', '나눔고딕', 'UnDotum')


def now_ms() -> float:
    return time.monotonic() * 1000.0


class SimDevices(Node):
    def __init__(self):
        super().__init__('sim_devices')
        self.declare_parameter('led_count', 8)
        self.declare_parameter('max_brightness', 100)
        self.declare_parameter('gz_world', 'twin_arena')
        self.declare_parameter('lamp_visual', 'robot_lamp_visual')
        self.declare_parameter('emotion_dir', '')

        self.lock = threading.Lock()
        self.led = LedStrip(count=self.get_parameter('led_count').value,
                            max_brightness=self.get_parameter('max_brightness').value)
        self.lamp = Lamp(start_ms=now_ms())
        self.emotion = 'happy'           # 실기 서버 시작 표정
        self.log = deque(maxlen=8)
        self.emotion_dir = self._find_emotion_dir()

        self.create_service(SetLed, 'set_led', self.on_led)
        self.create_service(SetBrightness, 'set_brightness', self.on_brightness)
        self.create_service(SetLamp, 'set_lamp', self.on_lamp)
        self.create_service(Emotion, 'set_emotion', self.on_emotion)

        world = self.get_parameter('gz_world').value
        self.lamp_visual = self.get_parameter('lamp_visual').value
        self.color_pub = self.create_publisher(MaterialColor, f'/world/{world}/material_color', 10)
        self.last_sent = None
        self.create_timer(0.05, self.push_lamp_to_gazebo)
        self.get_logger().info(f'가상 출력 장치 준비 (LED {self.led.count}개, Gazebo world "{world}")')

    def _find_emotion_dir(self) -> str:
        path = self.get_parameter('emotion_dir').value
        if path:
            return path
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('pinky_emotion'), 'emotion')
        except Exception:
            self.get_logger().warn('pinky_emotion 패키지가 없어 표정 GIF 없이 이름만 표시합니다')
            return ''

    def _record(self, ok: bool, text: str):
        self.log.appendleft(('OK ' if ok else 'ERR') + ' ' + text)
        (self.get_logger().info if ok else self.get_logger().warn)(text)

    # ---------- 서비스 (응답 문구는 실기 서버와 같다) ----------
    def on_led(self, req, res):
        with self.lock:
            res.success, res.message = self.led.command(req.command, list(req.pixels),
                                                        req.r, req.g, req.b)
            self._record(res.success, f'set_led {req.command}: {res.message}')
        return res

    def on_brightness(self, req, res):
        with self.lock:
            res.success, res.message = self.led.set_brightness(req.brightness)
            self._record(res.success, res.message)
        return res

    def on_lamp(self, req, res):
        c = req.color
        with self.lock:
            ok, msg = self.lamp.set(c.r, c.g, c.b, req.mode, req.time, now_ms())
            self._record(ok, msg)
        res.result = ok
        return res

    def on_emotion(self, req, res):
        ok, res.response = check_emotion(req.emotion)
        with self.lock:
            if ok:
                self.emotion = req.emotion
            self._record(ok, f'set_emotion {req.emotion}: {res.response}')
        return res

    # ---------- Gazebo ----------
    def push_lamp_to_gazebo(self):
        with self.lock:
            color = self.lamp.color_at(now_ms())
        if color == self.last_sent:
            return
        self.last_sent = color
        rgba = ColorRGBA(r=color[0] / 255, g=color[1] / 255, b=color[2] / 255, a=1.0)
        msg = MaterialColor()
        msg.entity = Entity(name=self.lamp_visual, type=Entity.VISUAL)
        msg.entity_match = MaterialColor.FIRST
        msg.ambient = msg.diffuse = msg.emissive = rgba
        self.color_pub.publish(msg)

    def snapshot(self):
        with self.lock:
            return (self.led.shown(), self.led.brightness, self.lamp.color_at(now_ms()),
                    Lamp.MODES[self.lamp.mode], self.lamp.time_ms, self.emotion, list(self.log))


# ---------------- 패널 창 (tkinter) ----------------
def run_panel(node: SimDevices):
    import tkinter as tk
    import tkinter.font as tkfont

    hexc = '#{:02x}{:02x}{:02x}'.format
    root = tk.Tk()
    # 창 제목은 WSLg/창 관리자가 그려서 한글이 깨질 수 있다 -> 영문
    root.title('Pinky Virtual Devices (simulation)')
    root.configure(bg='#1e1f22')
    fg, sub = '#e8e8e8', '#9aa0a6'
    families = set(tkfont.families(root))
    korean = next((f for f in KOREAN_FONTS if f in families), None)
    if korean is None:
        node.get_logger().warn('한글 폰트가 없어 패널 한글이 깨질 수 있습니다. '
                               'scripts/setup_env.sh --windows-fonts (WSL) 또는 '
                               'sudo apt install fonts-noto-cjk')
    font = (korean or 'TkDefaultFont', 11)

    def title(text):
        tk.Label(root, text=text, fg=fg, bg='#1e1f22', font=(font[0], 12, 'bold'),
                 anchor='w').pack(fill='x', padx=14, pady=(12, 2))

    title('LED 줄  (/set_led, /set_brightness)')
    n = node.led.count
    tk.Label(root, fg=sub, bg='#1e1f22', font=(font[0], 10), anchor='w', justify='left',
             text=(f'원 하나 = LED 하나, 아래 숫자 0~{n - 1} = set_pixel 의 pixels 번호.  '
                   'fill: 전체, clear: 전체 끄기\n'
                   '검은색 = 꺼짐. 실물에서 0번이 로봇의 어느 위치인지는 미확인')
             ).pack(fill='x', padx=14, pady=(0, 4))
    led_cv = tk.Canvas(root, width=max(360, n * 44 + 20), height=70, bg='#111214', highlightthickness=0)
    led_cv.pack(padx=14)
    leds = []
    for i in range(n):
        x = 20 + i * 44
        leds.append(led_cv.create_oval(x, 10, x + 34, 44, fill='#000000', outline='#3c4043', width=2))
        led_cv.create_text(x + 17, 58, text=str(i), fill=sub, font=(font[0], 9))
    led_info = tk.Label(root, fg=sub, bg='#1e1f22', font=font, anchor='w')
    led_info.pack(fill='x', padx=14)

    title('램프  (/set_lamp) — Gazebo 로봇 램프와 같은 색')
    lamp_row = tk.Frame(root, bg='#1e1f22')
    lamp_row.pack(fill='x', padx=14)
    lamp_cv = tk.Canvas(lamp_row, width=70, height=70, bg='#111214', highlightthickness=0)
    lamp_cv.pack(side='left')
    lamp = lamp_cv.create_oval(10, 10, 60, 60, fill='#000000', outline='#3c4043', width=2)
    lamp_info = tk.Label(lamp_row, fg=sub, bg='#1e1f22', font=font, justify='left')
    lamp_info.pack(side='left', padx=12)

    title('LCD 표정  (/set_emotion)')
    lcd = tk.Label(root, bg='#000000', fg=fg, font=(font[0], 14), width=26, height=6)
    lcd.pack(padx=14)

    title('최근 서비스 호출')
    log = tk.Label(root, fg=fg, bg='#111214', font=(font[0], 9), justify='left',
                   anchor='nw', width=70, height=8)
    log.pack(padx=14, pady=(0, 14), fill='x')

    gif = {'name': None, 'frames': [], 'i': 0}

    def load_gif(name):
        gif.update(name=name, frames=[], i=0)
        path = os.path.join(node.emotion_dir, f'{name}.gif') if node.emotion_dir else ''
        if not os.path.exists(path):
            return
        try:
            from PIL import Image, ImageSequence, ImageTk
            img = Image.open(path)
            for k, fr in enumerate(ImageSequence.Iterator(img)):
                if k % 2 == 0:   # 실기 기본 load_frame_skip=2
                    fr = fr.convert('RGB')
                    fr.thumbnail((260, 200))
                    gif['frames'].append(ImageTk.PhotoImage(fr))
        except Exception as e:  # PIL/ImageTk 미설치 등
            node.get_logger().warn(f'GIF 표시 실패({e}), 이름만 표시합니다')
            gif['frames'] = []

    def tick():
        if not rclpy.ok():
            root.destroy()
            return
        shown, bright, lamp_c, mode, t_ms, emo, lines = node.snapshot()
        for item, c in zip(leds, shown):
            led_cv.itemconfig(item, fill=hexc(*c))
        lit = '  '.join(f'{i}번 {c}' for i, c in enumerate(shown) if c != (0, 0, 0))
        led_info.config(text=f'밝기 {bright}/{node.led.max_brightness}   '
                             f'켜진 LED: {lit or "없음"}')
        lamp_cv.itemconfig(lamp, fill=hexc(*lamp_c))
        lamp_info.config(text=f'모드: {mode}   주기: {t_ms} ms\n목표색: {node.lamp.goal}')
        if emo != gif['name']:
            load_gif(emo)
        if gif['frames']:
            lcd.config(image=gif['frames'][gif['i'] % len(gif['frames'])], text='', width=260, height=200)
            gif['i'] += 1
        else:
            lcd.config(image='', text=f'( {emo} )')
        log.config(text='\n'.join(lines) or '아직 호출 없음')
        root.after(100, tick)    # 실기 표정 타이머와 같은 10 Hz

    root.protocol('WM_DELETE_WINDOW', lambda: (root.destroy()))
    tick()
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SimDevices()
    node.declare_parameter('gui', True)
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    try:
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if node.get_parameter('gui').value and has_display:
            run_panel(node)
        else:
            node.get_logger().info('패널 없이 실행 (gui:=false 또는 DISPLAY 없음)')
            spin.join()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
