"""정답 위치(/ground_truth)와 로봇이 추정한 위치(/odom, /amcl_pose)의 차이를 잰다.

  ros2 run pinky_sim_examples pose_error                      # 1초마다 출력, Ctrl+C 로 요약
  ros2 run pinky_sim_examples pose_error --ros-args -p duration:=120.0

/ground_truth 는 MuJoCo 시뮬레이터(pinky_mujoco)가 내보내는 실제 위치다. Gazebo 공식 브리지에는 없다.
비교는 모두 지도 좌표(= 시뮬레이터 world 좌표) 기준이다. 로봇이 원점에서 시작하고 AMCL 초기 위치도
(0, 0, 0)이라 세 좌표계가 처음에는 겹친다. odom 은 바퀴로만 계산하므로 주행할수록 오차가 쌓이고,
AMCL 은 라이다를 지도에 맞춰 그 오차를 바로잡는다 -> 둘의 오차 그래프 차이가 이 실습의 요점이다.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Pose2D = Tuple[float, float, float]      # x, y, yaw(rad)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pose_diff(est: Pose2D, truth: Pose2D) -> Tuple[float, float]:
    """(위치 오차 m, 방향 오차 deg)."""
    return (math.hypot(est[0] - truth[0], est[1] - truth[1]),
            abs(math.degrees(wrap(est[2] - truth[2]))))


@dataclass
class ErrorStats:
    pos: List[float] = field(default_factory=list)
    yaw: List[float] = field(default_factory=list)

    def add(self, pos: float, yaw: float):
        self.pos.append(pos)
        self.yaw.append(yaw)

    def summary(self) -> Optional[str]:
        if not self.pos:
            return None
        n = len(self.pos)
        return (f'평균 {sum(self.pos) / n * 100:.1f} cm / 최대 {max(self.pos) * 100:.1f} cm, '
                f'방향 평균 {sum(self.yaw) / n:.1f}° / 최대 {max(self.yaw):.1f}° ({n}회)')


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    class PoseError(Node):
        def __init__(self):
            super().__init__('pose_error')
            self.declare_parameter('period', 1.0)       # 출력 주기(s)
            self.declare_parameter('duration', 0.0)     # 0 이면 Ctrl+C 까지
            self.truth: Optional[Pose2D] = None
            self.odom: Optional[Pose2D] = None
            self.amcl: Optional[Pose2D] = None
            self.stats = {'odom': ErrorStats(), 'amcl': ErrorStats()}
            self.distance, self.last_xy = 0.0, None
            to2d = lambda p: (p.position.x, p.position.y, yaw_of(p.orientation))
            self.create_subscription(PoseStamped, 'ground_truth', lambda m: self.on_truth(to2d(m.pose)), 10)
            self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', to2d(m.pose.pose)), 10)
            self.create_subscription(
                PoseWithCovarianceStamped, 'amcl_pose', lambda m: setattr(self, 'amcl', to2d(m.pose.pose)),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(self.get_parameter('period').value, self.tick)
            self.t0 = self.get_clock().now()
            self.get_logger().info('정답 위치(/ground_truth)를 기다립니다. MuJoCo 시뮬레이션이 켜져 있어야 합니다')

        def on_truth(self, pose: Pose2D):
            if self.last_xy:
                self.distance += math.hypot(pose[0] - self.last_xy[0], pose[1] - self.last_xy[1])
            self.last_xy = pose[:2]
            self.truth = pose

        def tick(self):
            if self.truth is None:
                return
            parts = [f'주행 {self.distance:5.2f} m', f'정답 ({self.truth[0]:5.2f}, {self.truth[1]:5.2f})']
            for name in ('odom', 'amcl'):
                est = getattr(self, name)
                if est is None:
                    parts.append(f'{name} -')
                    continue
                pos, yaw = pose_diff(est, self.truth)
                self.stats[name].add(pos, yaw)
                parts.append(f'{name} {pos * 100:5.1f} cm {yaw:4.1f}°')
            print(' | '.join(parts), flush=True)
            limit = self.get_parameter('duration').value
            if limit and (self.get_clock().now() - self.t0).nanoseconds / 1e9 >= limit:
                raise SystemExit

        def report(self):
            print(f'--- 요약 (주행 {self.distance:.2f} m)')
            for name, st in self.stats.items():
                print(f'{name:5s}: {st.summary() or "받은 값 없음"}')

    rclpy.init(args=args)
    node = PoseError()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):   # Ctrl+C·종료 신호에도 요약을 남긴다
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
