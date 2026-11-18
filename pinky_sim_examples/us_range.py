"""Gazebo 초음파(us_scan, 좁은 라이다) -> 실기와 같은 us_sensor/range (sensor_msgs/Range).

실기 pinky_sensor_adc 의 메시지 형식(frame ultrasonic_link, fov 0.26, 0.02~3.0 m)을 그대로 쓴다.
값은 실기 계산식의 상한(0.97 m)에서 포화시킨다(ceiling 파라미터).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from pinky_sim_examples.sensors import REAL_US_CEILING, us_range


class UsRange(Node):
    def __init__(self):
        super().__init__('us_range')
        self.declare_parameter('ceiling', REAL_US_CEILING)
        self.ceiling = self.get_parameter('ceiling').value
        self.pub = self.create_publisher(Range, 'us_sensor/range', 10)
        self.create_subscription(LaserScan, 'us_scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan):
        msg = Range()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 3.0
        msg.range = float(us_range(scan.ranges, msg.min_range, self.ceiling))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UsRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
