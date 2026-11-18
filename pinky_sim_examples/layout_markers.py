"""레이아웃 YAML의 물체를 RViz 마커로 보여 준다 (Gazebo 창 없이 headless로 돌릴 때 필수).

  발행  /layout_markers  (visualization_msgs/MarkerArray, transient local -> RViz를 나중에 켜도 보임)
  구독  /sim_objects     (std_msgs/String, JSON) spawn_object 가 추가·삭제를 알려 준다
          {"op": "add", "object": {...레이아웃 물체 한 개...}}
          {"op": "remove", "name": "my_box"}

Gazebo world 좌표 = odom 좌표로 본다(로봇이 (0,0)에서 +X를 보고 소환되므로).
바퀴 미끄러짐으로 odom이 틀어지면 마커와 라이다 점이 조금 어긋날 수 있다.
"""
import json
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pinky_sim_examples.world_builder import parse_layout

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def shape_markers(shape, marker_id: int, frame_id: str, stamp):
    """Shape 하나 -> (모양 마커, 이름 마커)."""
    body = Marker()
    body.header.frame_id, body.header.stamp = frame_id, stamp
    body.ns, body.id, body.action = 'objects', marker_id, Marker.ADD
    body.pose.position.x, body.pose.position.y, body.pose.position.z = shape.x, shape.y, shape.z
    body.pose.orientation.z = math.sin(shape.yaw / 2)
    body.pose.orientation.w = math.cos(shape.yaw / 2)
    if shape.kind == 'box':
        body.type = Marker.CUBE
        body.scale.x, body.scale.y, body.scale.z = shape.size
    else:
        body.type = Marker.CYLINDER
        body.scale.x = body.scale.y = shape.size[0] * 2
        body.scale.z = shape.size[1]
    body.color.r, body.color.g, body.color.b = shape.color
    body.color.a = 0.55

    label = Marker()
    label.header = body.header
    label.ns, label.id, label.action = 'names', marker_id, Marker.ADD
    label.type = Marker.TEXT_VIEW_FACING
    label.pose.position.x, label.pose.position.y = shape.x, shape.y
    label.pose.position.z = shape.z * 2 + 0.04
    label.pose.orientation.w = 1.0
    label.scale.z = 0.035
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0
    label.text = shape.name
    return body, label


class LayoutMarkers(Node):
    def __init__(self):
        super().__init__('layout_markers')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value
        self.objects = {}            # name -> 레이아웃 물체 dict (origin 보정 전)
        self.origin = [0.0, 0.0]
        self.wall_height = 0.25
        self.pub = self.create_publisher(MarkerArray, 'layout_markers', LATCHED)
        self.create_subscription(String, 'sim_objects', self.on_event, 10)
        self.load(self.get_parameter('layout_file').value)
        self.publish()

    def load(self, path: str):
        if not path or not os.path.exists(path):
            self.get_logger().warn(f'레이아웃 파일 없음({path or "미지정"}): spawn_object 로 추가한 물체만 표시')
            return
        with open(path, encoding='utf-8') as f:
            layout = yaml.safe_load(f)
        self.origin = layout.get('origin', [0.0, 0.0])
        self.wall_height = layout.get('wall_height', 0.25)
        for i, obj in enumerate(layout.get('objects', [])):
            name = obj.get('name', f'{obj.get("type")}_{i}')
            self.objects[name] = {**obj, 'name': name}
        self.get_logger().info(f'레이아웃 {os.path.basename(path)}: 물체 {len(self.objects)}개 표시')

    def on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
            if ev['op'] == 'add':
                obj = ev['object']
                # spawn_object 는 Gazebo 좌표로 보낸다 -> 레이아웃 좌표로 되돌려 저장
                obj = self._to_layout_frame(obj)
                self.objects[obj['name']] = obj
            elif ev['op'] == 'remove':
                self.objects.pop(ev['name'], None)
            else:
                raise ValueError(ev['op'])
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'sim_objects 메시지 무시: {e}')
            return
        self.publish()

    def _to_layout_frame(self, obj: dict) -> dict:
        ox, oy = self.origin
        obj = dict(obj)
        for key in ('at', 'from', 'to'):
            if key in obj:
                obj[key] = [obj[key][0] + ox, obj[key][1] + oy]
        return obj

    def publish(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        layout = {'origin': self.origin, 'wall_height': self.wall_height,
                  'objects': list(self.objects.values())}
        try:
            shapes = parse_layout(layout)
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'레이아웃 오류: {e}')
            return
        stamp = self.get_clock().now().to_msg()
        for i, s in enumerate(shapes):
            arr.markers.extend(shape_markers(s, i, self.frame_id, stamp))
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LayoutMarkers()
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
