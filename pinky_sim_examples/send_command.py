"""명령을 보내고 끝날 때까지 결과를 보여주는 도구.

  ros2 run pinky_sim_examples send "forward 0.3; turn 90"

`ros2 topic pub --once` 는 구독자와 연결되기 전에 끝나 메시지가 사라질 때가 있다.
이 도구는 command_driver 와 연결될 때까지 기다린 뒤 보내고, /command_status 를 출력한다.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

FINAL = ('모든 명령 완료', 'blocked', 'error', 'stopped')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    text = ' '.join(a for a in argv if not a.startswith('--ros-args'))
    if not text:
        print('사용법: ros2 run pinky_sim_examples send "forward 0.3; turn 90"')
        return 1
    rclpy.init()
    node = Node('send_command')
    finished = []
    # "speed 0.2" 처럼 움직임이 없는 명령은 'done' 이 오지 않으므로 'speed:' 알림에서 끝낸다
    words = text.lower().replace(';', ' ').split()
    speed_only = all(w in ('speed',) or w.replace('.', '', 1).isdigit() for w in words)

    def on_status(msg: String):
        print(msg.data, flush=True)
        if any(k in msg.data for k in FINAL) or (speed_only and msg.data.startswith('speed:')):
            finished.append(True)

    pub = node.create_publisher(String, 'pinky_command', 10)
    node.create_subscription(String, 'command_status', on_status, 10)

    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 or node.count_publishers('command_status') == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            print('command_driver 를 찾지 못했습니다. launch 가 켜져 있나요?')
            node.destroy_node()
            rclpy.shutdown()
            return 2
    time.sleep(0.3)   # 상태 구독 연결 여유
    pub.publish(String(data=text))

    deadline = time.monotonic() + 120.0
    try:
        while not finished and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pub.publish(String(data='stop'))
        time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
