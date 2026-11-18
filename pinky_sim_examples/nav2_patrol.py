"""Nav2 실습장 목표 지점을 차례로 보내는 예제 (nav2_simple_commander).

  ros2 run pinky_sim_examples nav2_patrol                 # G2 -> G3 -> G4 -> G1
  ros2 run pinky_sim_examples nav2_patrol G5              # 낮은 턱 위 (관찰용)
  ros2 run pinky_sim_examples nav2_patrol G3 G1 --loop 3  # 3바퀴

먼저 띄울 것:
  ros2 launch pinky_sim_examples twin_arena.launch.py world_name:=nav2_practice.world \
      gz_world:=nav2_practice driver:=false
  ros2 launch pinky_navigation gz_bringup_launch.xml \
      map:=$(ros2 pkg prefix pinky_sim_examples)/share/pinky_sim_examples/maps/nav2_practice.yaml

driver:=false 로 command_driver 를 끄는 이유: 두 노드가 같은 /cmd_vel 을 내면 서로 싸운다.
좌표는 layouts/nav2_practice.yaml 맨 위 주석과 같다(map 좌표 = Gazebo 좌표).
"""
import argparse
import math
import sys
import time

GOALS = {
    'G1': (0.0, 0.0, 0.0),        # 출발점
    'G2': (-0.1, 1.4, 1.57),      # 왼쪽 위: 문을 지나 바로 왼쪽으로
    'G3': (2.2, 1.1, 0.0),        # 오른쪽 위: 지그재그 통과
    'G4': (2.3, -0.3, 0.0),       # 오른쪽 아래: 기둥·상자 사이 (좁은 틈을 쓸 수도 있음)
    'G5': (0.55, -0.3, 0.0),      # 낮은 턱 위: 지도에 없는 장애물
}


def make_pose(nav, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait_active(node, names=('amcl', 'bt_navigator'), timeout=120.0):
    """lifecycle 노드들이 active 가 될 때까지 기다린다.

    BasicNavigator.waitUntilNav2Active() 는 get_state 응답이 한 번 사라지면 끝없이 기다린다
    (WSL 에서 실제로 겪음). 여기서는 요청마다 2초 제한을 두고 다시 묻는다.
    """
    import rclpy
    from lifecycle_msgs.srv import GetState
    deadline = time.monotonic() + timeout
    for name in names:
        cli = node.create_client(GetState, f'{name}/get_state')
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError(f'{name} 이 {timeout:.0f}s 안에 active 가 되지 않았습니다. '
                                   'gz_bringup_launch 가 켜져 있나요?')
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
                if fut.done() and fut.result() and fut.result().current_state.label == 'active':
                    print(f'{name}: active', flush=True)
                    break
            time.sleep(0.5)
        node.destroy_client(cli)


def main(argv=None):
    a = argparse.ArgumentParser(description='Nav2 실습장 목표 지점 순회')
    # nargs='*' 에 choices 를 같이 쓰면 기본값 목록 전체를 검사해 실패한다 -> 직접 검사
    a.add_argument('goals', nargs='*', help='G1~G5 (기본: G2 G3 G4 G1)')
    a.add_argument('--loop', type=int, default=1, help='반복 횟수')
    a.add_argument('--timeout', type=float, default=150.0, help='목표 하나당 제한 시간(s)')
    args = a.parse_args([x for x in (sys.argv[1:] if argv is None else argv) if not x.startswith('--ros-args')])
    args.goals = [g.upper() for g in args.goals] or ['G2', 'G3', 'G4', 'G1']
    bad = [g for g in args.goals if g not in GOALS]
    if bad:
        a.error(f'모르는 목표 {bad}. 가능: {", ".join(sorted(GOALS))}')

    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    rclpy.init()
    nav = BasicNavigator()
    wait_active(nav)          # amcl(위치 추정)과 bt_navigator(목표 처리)가 켜질 때까지
    ok = 0
    total = len(args.goals) * args.loop
    for _ in range(args.loop):
        for name in args.goals:
            x, y, yaw = GOALS[name]
            t0 = time.monotonic()
            nav.goToPose(make_pose(nav, x, y, yaw))
            while not nav.isTaskComplete():          # 내부에서 0.1 s 씩 기다리며 결과를 확인한다
                if time.monotonic() - t0 > args.timeout:
                    nav.cancelTask()
                    break
            result = nav.getResult()
            ok += result == TaskResult.SUCCEEDED
            print(f'{name} ({x}, {y}) -> {result.name} ({time.monotonic() - t0:.1f} s)', flush=True)
    print(f'성공 {ok}/{total}')
    nav.destroy_node()
    rclpy.shutdown()
    return 0 if ok == total else 1


if __name__ == '__main__':
    sys.exit(main())
