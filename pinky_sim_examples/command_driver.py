"""문장 명령으로 Pinky를 움직이는 노드.

구독  /pinky_command  (std_msgs/String)   예) "forward 0.5; turn 90"
      /odom           (nav_msgs/Odometry) 이동 거리·회전 각도 확인
      /scan           (sensor_msgs/LaserScan) 진행 방향 장애물 감지
발행  /cmd_vel        (geometry_msgs/Twist)
      /command_status (std_msgs/String)   accepted / done / blocked / error 알림

실물 Pinky와 Gazebo가 같은 토픽 이름을 쓰므로 이 노드는 둘 다에서 그대로 동작한다.
"""
import math
import time
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from pinky_sim_examples.commands import (Step, parse, rate_hz, sector_min, wrap_angle,
                                         yaw_from_quaternion)

SCAN_SAMPLES = 40   # 시작 후 이만큼 /scan 을 받아 주기를 한 번 점검


class CommandDriver(Node):
    def __init__(self):
        super().__init__('command_driver')
        self.declare_parameter('linear_speed', 0.10)      # m/s
        self.declare_parameter('angular_speed', 0.8)      # rad/s (최대)
        self.declare_parameter('stop_distance', 0.20)     # m, 라이다 기준 거리
        self.declare_parameter('sector_half_width_deg', 20.0)
        self.declare_parameter('lidar_mount_yaw', math.pi)  # rplidar_link rpy="0 0 pi"
        self.declare_parameter('distance_tolerance', 0.01)  # m
        self.declare_parameter('angle_tolerance_deg', 2.0)
        self.declare_parameter('expected_scan_hz', 10.0)     # 라이다 설정 주기
        self.declare_parameter('scan_rate_warn_ratio', 0.8)  # 이 비율 밑이면 경고

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.half_width = math.radians(self.get_parameter('sector_half_width_deg').value)
        self.mount_yaw = self.get_parameter('lidar_mount_yaw').value
        self.dist_tol = self.get_parameter('distance_tolerance').value
        self.ang_tol = math.radians(self.get_parameter('angle_tolerance_deg').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'command_status', 10)
        self.create_subscription(String, 'pinky_command', self.on_command, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        self.queue: deque = deque()
        self.active: Optional[Step] = None
        self.start_xy = (0.0, 0.0)
        self.last_yaw = 0.0
        self.turned = 0.0           # 현재 회전 명령에서 누적한 각도
        self.pose: Optional[tuple] = None   # (x, y, yaw)
        self.front = self.rear = math.inf
        self.scan_stamps, self.scan_walls = [], []
        self.scan_checked = False
        self.create_timer(0.05, self.control)   # 20 Hz
        self.get_logger().info("명령 대기: ros2 topic pub --once /pinky_command "
                               "std_msgs/String \"{data: 'forward 0.3'}\"")

    # ---------- 입력 ----------
    def on_command(self, msg: String):
        try:
            steps = parse(msg.data)
        except ValueError as e:
            self.report(f'error: {e}')
            return
        if steps[0].kind == 'stop':
            self.halt('stopped: 남은 명령 취소')
            return
        self.queue.extend(steps)
        self.report(f'accepted: {msg.data} (대기 {len(self.queue)}단계)')

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_scan(self, msg: LaserScan):
        if not self.scan_checked:
            self.check_scan_rate(msg)
        args = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.front = sector_min(*args, 0.0, self.half_width, self.mount_yaw, msg.range_min)
        self.rear = sector_min(*args, math.pi, self.half_width, self.mount_yaw, msg.range_min)

    def check_scan_rate(self, msg: LaserScan):
        """센서 시각(header.stamp) 기준 주기와 실제 시간 기준 주기를 한 번 재서 알린다.

        센서 시각 기준이 낮으면 라이다 자체가 늦게 나온다 -> 정지가 늦어질 수 있다.
        실제 시간 기준만 낮으면 시뮬레이션이 실제보다 느리게 도는 것(RTF < 1)이다.
        """
        st = msg.header.stamp
        self.scan_stamps.append(st.sec + st.nanosec * 1e-9)
        self.scan_walls.append(time.monotonic())
        if len(self.scan_stamps) < SCAN_SAMPLES:
            return
        self.scan_checked = True
        sensor_hz, wall_hz = rate_hz(self.scan_stamps), rate_hz(self.scan_walls)
        expected = self.get_parameter('expected_scan_hz').value
        ratio = self.get_parameter('scan_rate_warn_ratio').value
        rtf = wall_hz / sensor_hz if sensor_hz else 0.0
        text = (f'/scan 주기: 센서 시각 {sensor_hz:.1f} Hz, 실제 시간 {wall_hz:.1f} Hz '
                f'(설정 {expected:.0f} Hz, 시뮬레이션 속도 약 {rtf:.2f}배)')
        if sensor_hz < expected * ratio:
            self.get_logger().warn(text + ' -> 라이다가 늦게 나옵니다. stop_distance 를 늘리거나 '
                                   '속도를 낮추세요')
        else:
            self.get_logger().info(text)
            if wall_hz < expected * ratio:
                self.get_logger().info('실제 시간 기준만 낮음: 시뮬레이션이 느리게 도는 것이고 '
                                       '주행 판단에는 영향이 적습니다')

    # ---------- 제어 루프 ----------
    def control(self):
        if self.pose is None:
            return
        if self.active is None and not self.start_next():
            return
        step, (x, y, yaw) = self.active, self.pose
        twist = Twist()
        if step.kind == 'move':
            direction = 1.0 if step.value > 0 else -1.0
            obstacle = self.front if direction > 0 else self.rear
            if obstacle < self.stop_distance:
                self.halt(f'blocked: {"앞" if direction > 0 else "뒤"} {obstacle:.2f} m에 장애물')
                return
            remaining = abs(step.value) - math.hypot(x - self.start_xy[0], y - self.start_xy[1])
            if remaining <= self.dist_tol:
                self.finish_step()
                return
            # 끝에 가까우면 감속(최저 0.03 m/s)
            twist.linear.x = direction * max(0.03, min(self.linear_speed, remaining * 1.5))
        elif step.kind == 'turn':
            self.turned += wrap_angle(yaw - self.last_yaw)
            self.last_yaw = yaw
            remaining = step.value - self.turned
            if abs(remaining) <= self.ang_tol:
                self.finish_step()
                return
            speed = max(0.15, min(self.angular_speed, abs(remaining) * 2.0))
            twist.angular.z = math.copysign(speed, remaining)
        self.cmd_pub.publish(twist)

    def start_next(self) -> bool:
        while self.queue:
            step = self.queue.popleft()
            if step.kind == 'speed':
                self.linear_speed = step.value
                self.report(f'speed: {step.value:.2f} m/s')
                continue
            self.active = step
            x, y, yaw = self.pose
            self.start_xy, self.last_yaw, self.turned = (x, y), yaw, 0.0
            return True
        return False

    def finish_step(self):
        self.cmd_pub.publish(Twist())
        s = self.active
        what = f'move {s.value:+.2f} m' if s.kind == 'move' else f'turn {math.degrees(s.value):+.0f} deg'
        self.active = None
        self.report(f'done: {what}' + ('' if self.queue else ' (모든 명령 완료)'))

    def halt(self, reason: str):
        self.queue.clear()
        self.active = None
        self.cmd_pub.publish(Twist())
        self.report(reason)

    def report(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = CommandDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
