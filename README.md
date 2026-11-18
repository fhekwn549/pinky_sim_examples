# pinky_sim_examples — 로봇 없이 Pinky Pro 실습하기

공식 [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) 의 Gazebo 시뮬레이션 위에 네 가지를 얹은 예제 패키지다.

1. **벽·물체 만들기**: 레이아웃 파일(YAML)에 "바닥 중심 좌표 + 크기"만 적으면 Gazebo world가 생긴다. 기본 맵은 2.0 × 1.5 m 경기장(칸막이·선반)이다.
2. **문장 명령 주행**: `forward 0.5; turn 90` 같은 문장을 보내면 odom으로 거리·각도를 확인하며 움직이고, 앞이 막히면 `/scan`을 보고 멈춘다.
3. **가상 출력 장치**: `/set_led`·`/set_lamp`·`/set_emotion` 서비스를 받아 패널 창과 Gazebo 램프 색으로 보여 준다.
4. **센서 붙이기**: 카메라·포인트클라우드(RealSense D455 가정)·초음파·IMU를 Pinky에 붙이는 방법

그림·실행 결과까지 담은 전체 가이드는 [docs/pinky_sim_guide.html](docs/pinky_sim_guide.html)이다(내려받아 브라우저로 연다). 벽·물체 만들기, 색·텍스처, SLAM 지도, Nav2 실습장, 센서 붙이기를 다룬다.

`cmd_vel`, `odom`, `scan` 토픽 이름이 실물 Pinky와 같아서, 여기서 만든 명령 노드를 **실물에 그대로** 쓸 수 있다.

---

## 0. 준비 (Ubuntu 24.04 + ROS 2 Jazzy, 최초 1회)

```bash
mkdir -p ~/pinky_pro/src && cd ~/pinky_pro/src
git clone https://github.com/pinklab-art/pinky_pro.git
git clone https://github.com/fhekwn549/pinky_sim_examples.git
cd ~/pinky_pro
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 0-1. 실행 환경 설정 (빌드 후 한 번, 꼭 먼저)

```bash
bash ~/pinky_pro/src/pinky_sim_examples/scripts/setup_env.sh --bashrc
source ~/.bashrc
```

WSL인지 네이티브 Ubuntu인지 스스로 판별하고, 환경을 점검한 뒤 설정 파일 두 개를 만든다.

| 파일 | 내용 | 쓰는 곳 |
|---|---|---|
| `~/.config/pinky_sim_examples/env.sh` | `ROS_DOMAIN_ID`, 필요 시 `LIBGL_ALWAYS_SOFTWARE` | 터미널(`--bashrc`면 자동 source) |
| `~/.config/pinky_sim_examples/runtime.yaml` | 환경별 노드 파라미터(`stop_distance` 등) | launch가 자동으로 읽음 |

- `[확인]` 줄이 나오면 그 아래 `->` 명령을 실행한다. sudo나 Windows 설정이 필요한 것은 스크립트가 직접 바꾸지 않는다.
- WSL에서 패널 한글이 깨지면: `setup_env.sh --windows-fonts` (Windows 맑은 고딕을 연결, sudo 불필요)
- 실물 Pinky와 함께 쓸 때는 로봇과 같은 번호로: `setup_env.sh --domain <번호>`
- 설정 파일은 git 저장소 밖(홈 폴더)에 있어서, 사람마다 값이 달라도 코드에는 섞이지 않는다.
- 이 단계를 건너뛰면 launch가 기본값으로 실행하면서 경고를 띄운다.

## 1. 실행

```bash
ros2 launch pinky_sim_examples twin_arena.launch.py
```

Gazebo 서버(기본 경기장) + `command_driver` + 가상 출력 장치 패널 + RViz가 함께 뜬다. 로봇은 경기장 앞쪽 통로에서 +X(오른쪽)를 보고 시작한다.

**기본은 Gazebo 창 없이(headless) 돌고, 보는 것은 RViz에서 한다.** 창을 그리는 부하가 없어 WSL·노트북에서도 가볍다. RViz 화면 읽는 법:

| RViz에 보이는 것 | 뜻 |
|---|---|
| 반투명 상자·원기둥 + 이름 | 레이아웃 YAML의 물체 (`layout_markers` 노드가 그림) |
| 빨간 점 | 라이다 `/scan` |
| 노란 부채꼴 | 초음파 `/us_sensor/range` |
| Camera 창 | 전방 카메라 `/camera/image_raw` |

상자는 있는데 빨간 점이 없으면 **라이다가 못 보는 물체**다(3장). Gazebo 창이 필요할 때(GUI로 물체 그리기, Gazebo 속 램프 색 보기)는 `gz_gui:=true`를 붙인다.

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `gz_gui` | `false` | Gazebo 창 |
| `world_name` / `gz_world` | `twin_arena.world` / `twin_arena` | world 파일 / 그 안의 world 이름 |
| `rviz`, `driver`, `devices` | `true` | RViz, 명령 주행, 가상 출력 장치 |
| `d455` | `false` | RealSense D455 장착 가정(튜토리얼) |
| `led_count` | `8` | 가상 LED 개수 |

준비된 world: `twin_arena`(기본 경기장 2.0 × 1.5 m), `my_room`(연습용 2 × 2 m), `scan_room`(D455 스캔 실습용 3 × 3 m 정사각형 방).

다른 터미널에서 명령을 보낸다.

```bash
source ~/pinky_pro/install/setup.bash
ros2 run pinky_sim_examples send "forward 1.0"
```

```
accepted: forward 1.0 (대기 1단계)
blocked: 앞 0.20 m에 장애물        <- 0.6 m 앞 주황 상자(demo_obstacle)에서 정지
```

### 명령 문법 (`;` 로 이어 쓰기)

| 명령 | 뜻 | 예 |
|---|---|---|
| `forward <m>` / `back <m>` | 앞·뒤로 m 만큼 | `forward 0.5` |
| `turn <deg>` | 제자리 회전(+ 왼쪽, - 오른쪽) | `turn -90` |
| `square <m>` | 한 변 m 정사각형 | `square 0.3` |
| `speed <m/s>` | 직진 속도(0.02~0.25, 실기 상한 약 0.28 m/s) | `speed 0.05` |
| `stop` | 즉시 정지, 남은 명령 취소 | `stop` |

```bash
ros2 run pinky_sim_examples send "turn 90; forward 0.2; turn -90"
ros2 run pinky_sim_examples send "speed 0.2; square 0.25"
```

`send` 대신 표준 CLI로 보내도 된다. 이때 `--keep-alive 1`을 꼭 붙인다(없으면 연결 전에 끝나 메시지가 자주 사라진다).

```bash
ros2 topic pub --once --keep-alive 1 /pinky_command std_msgs/String "{data: 'forward 0.3'}"
ros2 topic echo /command_status
```

---

## 2. Gazebo에 벽·물체를 만드는 4가지 방법

### 방법 ① SDF로 직접 쓰기 — 원리

Gazebo world는 `<model>`의 모음이다. 벽 하나는 이렇게 생겼다.

```xml
<model name="wall_back">
  <static>true</static>                         <!-- 안 움직이는 물체 -->
  <pose>0.2 1.385 0.125 0 0 0</pose>            <!-- x y z roll pitch yaw (물체 "중심") -->
  <link name="link">
    <collision name="collision">                <!-- 부딪힘·라이다가 보는 모양 -->
      <geometry><box><size>2.04 0.02 0.25</size></box></geometry>
    </collision>
    <visual name="visual">                      <!-- 눈에 보이는 모양 -->
      <geometry><box><size>2.04 0.02 0.25</size></box></geometry>
      <material><diffuse>0.55 0.63 0.67 1</diffuse></material>
    </visual>
  </link>
</model>
```

주의할 점 세 가지.

- **pose의 z는 물체 중심**이다. 높이 0.25 m 벽이면 z = 0.125. (바닥에 놓으려면 높이의 절반)
- **collision이 없으면** 로봇이 통과하고 라이다에도 안 보인다. visual만 있으면 "유령 벽"이다.
- world에 `gz-sim-sensors-system` 플러그인이 없으면 `/scan`이 나오지 않는다. 새 world는 `pinky_gz_sim/worlds/empty.world`를 복사해서 시작하는 게 안전하다.

### 방법 ② 레이아웃 YAML → world 생성 — 이 패키지의 방식

SDF를 손으로 쓰면 길고, 벽마다 중심·각도 계산이 귀찮다. `layouts/*.yaml`에 이렇게만 적는다.

```yaml
world_name: my_room
origin: [0.0, 0.0]          # 이 점이 Gazebo (0,0) = 로봇 소환 위치
wall_height: 0.25
objects:
  - {name: north, type: wall, from: [-1.0, 1.0], to: [1.0, 1.0]}          # 두 점을 잇는 벽
  - {name: box1,  type: box, size: [0.1, 0.1, 0.2], at: [0.5, 0.0]}       # [X폭, Y깊이, 높이], 바닥 중심
  - {name: pole,  type: cylinder, radius: 0.05, height: 0.3, at: [-0.5, 0.5]}
  - {name: can,   type: cylinder, radius: 0.03, height: 0.16, at: [0.3, -0.3], static: false, mass: 0.1}  # 밀리는 물체
```

```bash
cd ~/pinky_pro/src/pinky_sim_examples
ros2 run pinky_sim_examples make_world layouts/my_room.yaml -o worlds/my_room.world
cd ~/pinky_pro && colcon build --packages-select pinky_sim_examples
ros2 launch pinky_sim_examples twin_arena.launch.py world_name:=my_room.world
```

`wall`은 길이·각도·중심을 자동 계산하고, `box`·`cylinder`는 z를 높이의 절반으로 넣어 준다. `layouts/my_room.yaml`에 실습용 TODO가 있다.

### 방법 ③ 실행 중에 물체 추가·삭제 — world 파일은 그대로

Gazebo를 끄지 않고 장애물을 놓아 볼 때 쓴다.

```bash
# 로봇 바로 앞에 상자
ros2 run pinky_sim_examples spawn_object box --size 0.1 0.3 0.2 --at 0.3 0 --name my_box
# 두 점을 잇는 벽
ros2 run pinky_sim_examples spawn_object wall --from 0.5 -0.3 --to 0.5 0.3 --name gate
# 삭제
ros2 run pinky_sim_examples spawn_object remove --name my_box
# 어떤 SDF가 만들어지는지만 보기
ros2 run pinky_sim_examples spawn_object box --at 0.3 0 --dry-run
```

추가·삭제하면 RViz 마커도 같이 바뀐다(`layout_markers` 노드에 알림). 내부적으로는 표준 도구 `ros2 run ros_gz_sim create -world twin_arena -string '<sdf>…</sdf>'`와 `gz service -s /world/twin_arena/remove`를 호출한다.

### 방법 ④ Gazebo GUI에서 그리기 (`gz_gui:=true` 필요)

Gazebo 상단 툴바의 **Box / Cylinder / Sphere** 아이콘을 누르고 바닥을 클릭하면 물체가 생긴다. 오른쪽 **Component Inspector**에서 pose·크기를 바꾼다. 마음에 들면 왼쪽 위 메뉴 → **Save world as**로 `.world`를 저장한다.
빠르게 모양을 잡을 때 좋지만 저장 파일이 길고 숫자가 지저분해서, 다시 쓸 맵은 ②로 관리하는 걸 권한다.

### MuJoCo에서는

같은 world 파일을 MuJoCo도 읽는다(3-2절). 다만 MuJoCo는 시작할 때 world와 로봇을 모델 하나로 묶어 두므로, 물체는 실행 전에만 정할 수 있다. 시험용 world를 MuJoCo로 띄워 확인한 결과다.

| Gazebo에서 한 것 | MuJoCo | 대신 이렇게 |
|---|---|---|
| SDF 직접 쓰기, 레이아웃 YAML → `make_world` | 됨 | 같은 world 를 `mujoco_sim.launch.py world_name:=…` 로 실행 |
| 실행 중 추가·삭제 (`spawn_object`) | 안 됨 | 레이아웃을 고치고 `make_world` 후 다시 실행 |
| Gazebo 창에서 그리기 | 안 됨 | MuJoCo 창에는 도형 추가 도구가 없다 |
| 단색 (`color`) | 됨 | 그대로 반영 |
| 이미지 텍스처 (`texture`) | 상자·원기둥은 무시 | 흰 상자로 보인다 |
| 밀리는 물체 (`static: false`) | 고정됨 | 로봇이 캔을 밀지 못하고 막힌다(0.268 m에서 정지) |

MuJoCo 전용 YAML 상자 목록(`pinky_mujoco/worlds/room.yaml`)도 있지만 형식이 달라 Gazebo에서는 못 쓴다. 두 시뮬레이터에서 같은 맵을 쓰려면 `make_world`로 만든 SDF를 쓴다.

---

## 3. 꼭 관찰해 볼 것: "라이다는 낮은 물체를 못 본다"

Pinky의 라이다(`rplidar_link`)는 바닥에서 약 **0.125 m** 높이에서 수평으로 한 줄만 쏜다.

- 기본 경기장의 선반(높이 0.11 m)은 RViz `/scan`에 **안 찍힌다**. Gazebo에는 보이는데 라이다 점이 없다.
- 그래서 경기장 벽은 라이다보다 넉넉히 높은 0.25 m로 만들었다(`twin_arena.yaml` 주석 참고).
- 실물 현장에서도 낮은 턱·박스는 SLAM 지도와 Nav2 장애물에서 빠진다. 프로젝트에서 맵을 만들 때 고려할 점이다.

또 하나: 라이다가 로봇에 **180° 돌아가** 붙어 있다(`rpy="0 0 pi"`). `/scan`의 각도 0은 로봇 **뒤쪽**이다. `command_driver`의 `lidar_mount_yaw` 파라미터가 이것을 보정한다.

---

## 3-1. Nav2 실습장 (`nav2_practice`)

기본 경기장은 Nav2 연습에는 단조롭다. `nav2_practice`는 Nav2 실습용으로 따로 만든 3.0 × 2.4 m, 방 2개짜리 맵이다. 공식 `pinky_navigation` 설정을 **고치지 않고** 그대로 돌린다.

```
   y 1.8  +------------------------------------------+
          |  G2      |          [선반]      G3       |  위쪽 방: 지그재그 벽 2개
          |          |    |                          |
   y 0.7  +------[  문 0.8 m  ]-----------+[0.2]+----+  가운데 벽: 문 + 좁은 틈
          |              O 기둥    [상자]            |
   y 0    |  >로봇  [낮은 턱]        O 기둥    G4    |  아래쪽 방
   y-0.6  +------------------------------------------+
        x -0.4                                    x 2.6
```

### 실행

지도는 레이아웃에서 계산해 둔 `maps/nav2_practice.yaml`을 쓴다(SLAM으로 직접 만들어도 된다. 아래 참고).

```bash
# 터미널 1: 시뮬레이션 (command_driver 는 끈다. Nav2와 /cmd_vel 을 두고 싸우지 않게)
ros2 launch pinky_sim_examples twin_arena.launch.py \
    world_name:=nav2_practice.world gz_world:=nav2_practice driver:=false

# 터미널 2: Nav2 (공식 launch 그대로, 지도만 지정)
ros2 launch pinky_navigation gz_bringup_launch.xml \
    map:=$(ros2 pkg prefix pinky_sim_examples)/share/pinky_sim_examples/maps/nav2_practice.yaml

# 터미널 3: RViz 에서 보며 목표 보내기 (2D Goal Pose 버튼)
ros2 launch pinky_navigation gz_nav2_view.launch.xml

# 또는 코드로 목표 순회 (nav2_simple_commander)
ros2 run pinky_sim_examples nav2_patrol              # G2 -> G3 -> G4 -> G1
ros2 run pinky_sim_examples nav2_patrol G5           # 낮은 턱 위
```

공식 `nav2_params.yaml`의 AMCL 초기 위치가 `(0, 0, 0)`이고, 이 맵의 지도 좌표도 Gazebo 좌표 그대로라서 **2D Pose Estimate를 따로 하지 않아도 된다**.

### 목표 지점과 시험 결과 (WSL, 2026-11-18, 공식 파라미터 그대로)

| 목표 | 좌표 (x, y, yaw) | 보는 것 | 결과 |
|---|---|---|---|
| G2 | (-0.1, 1.4, 1.57) | 문을 지나 바로 왼쪽으로 | 성공 19.9 s |
| G3 | (2.2, 1.1, 0) | 지그재그 통과(직선거리보다 먼 경로) | 성공 25.8 s |
| G4 | (2.3, -0.3, 0) | 기둥·상자 사이, 좁은 틈을 쓸지 | 성공 39.6 s |
| G1 | (0, 0, 0) | 출발점 복귀 | 성공, AMCL 기준 오차 0.05 m |
| G5 | (0.55, -0.3, 0) | 지도에 없는 낮은 턱 위 | **턱 앞에서 멈췄는데 "성공"** (허용 오차 0.25 m 안) |

### 실험 거리

| 바꿔 볼 것 | 어디서 | 관찰 |
|---|---|---|
| 문 폭 0.8 → 0.6 / 0.4 m | `layouts/nav2_practice.yaml` 의 `mid_wall_1` | 이 맵을 설계할 때 실제로 겪은 실패. 0.4·0.6 m에서는 G2로 갈 때 경로가 문틀 모서리를 바짝 돌아 `collision ahead`로 멈췄다 |
| `inflation_radius` 0.15 → 0.25 / 0.10 | `pinky_navigation/params/nav2_params.yaml` | 0.2 m 좁은 틈은 공식 설정에서 경계값이다. 시험에서 한 번은 돌아가고(3.74 m) 한 번은 틈으로 지나갔다(1.67 m). 키우면 항상 피하는지, 줄이면 항상 지나가는지 |
| `xy_goal_tolerance` 0.25 → 0.05 | 같은 파일 `general_goal_checker` | G5가 "성공"에서 실패로 바뀌는지. 허용 오차가 크면 막혀도 성공으로 보고된다 |
| 실행 중 장애물 추가 | `spawn_object box --at 1.5 0 --world nav2_practice` | 저장된 지도에 없는 물체를 라이다가 보고 지역 코스트맵이 피해 가는지 |

파라미터를 바꿀 때는 공식 파일을 직접 고치지 말고 복사해서 `params_file:=<복사본>` 인자로 넘긴다.

### 지도 만들기 두 가지

| 방법 | 명령 | 언제 |
|---|---|---|
| 레이아웃에서 계산 | `ros2 run pinky_sim_examples make_map layouts/nav2_practice.yaml -o maps/nav2_practice` | Nav2 연습을 바로 시작할 때. 라이다보다 낮은 물체는 넣지 않아 SLAM 지도와 같은 기준이다 |
| SLAM으로 직접 | `gz_map_building.launch.xml` → 주행 → `map_saver_cli` (1장 참고) | 실물과 같은 과정을 연습할 때 |

---

## 3-2. MuJoCo로 실행하기

공식 repo에는 시뮬레이터가 두 개 있다. 공식 `bringup_sim.launch.xml`의 기본값은 Gazebo가 아니라 **MuJoCo**다(`sim:=gz`를 붙여야 Gazebo).
이 패키지의 앞 절은 Gazebo 기준이고, 아래는 같은 맵·같은 Nav2를 MuJoCo로 돌리는 방법이다. 그림이 들어간 상세판은 `docs/pinky_sim_guide.html`의 "MuJoCo로 실습하기" 섹션에 있다.

| 항목 | Gazebo | MuJoCo |
|---|---|---|
| 시뮬레이션 속도(이 PC, WSL) | 실제 시간의 약 0.67배 | 약 1.0~1.08배 |
| GPU | 센서 렌더링에 사용 | 필요 없음 |
| 정답 위치 | ROS로 나오지 않음 | `/ground_truth` 50 Hz |
| 모터·속도 | 최대 1.0 m/s | 실물 XL330 가속, 최대 0.25 m/s |
| 라이다 | 640점, 잡음 있음 | 360점, 잡음 없음 |
| 그 밖의 센서·실행 중 물체 추가 | 됨 | 안 됨 |

```bash
# 최초 1회 (공식 README 버전)
pip install --user --break-system-packages "mujoco==3.6.0" "trimesh>=4,<5" "pycollada>=0.9,<1"

# 이 패키지 world 로 실행 (기본 nav2_practice, command_driver·RViz 포함)
ros2 launch pinky_sim_examples mujoco_sim.launch.py
ros2 launch pinky_sim_examples mujoco_sim.launch.py world_name:=twin_arena.world viewer:=true

# Nav2 는 Gazebo 와 같은 명령 (command_driver 는 끈다)
ros2 launch pinky_sim_examples mujoco_sim.launch.py driver:=false
ros2 launch pinky_navigation gz_bringup_launch.xml     map:=$(ros2 pkg prefix pinky_sim_examples)/share/pinky_sim_examples/maps/nav2_practice.yaml
ros2 run pinky_sim_examples nav2_patrol

# 정답 위치와 odom·AMCL 오차 비교 (MuJoCo 전용)
ros2 run pinky_sim_examples pose_error
```

**확인한 것 (WSL, MuJoCo 3.6.0, 2026-11-18)**: `make_world`로 만든 world 로딩, 문장 명령 주행, SLAM 지도 저장, Nav2 + `nav2_patrol` 4/4, 카메라 저장(640 × 360), 물리 설정 교체(`physics_config`). `spawn_object`, 초음파·IMU·D455, Gazebo 램프 색은 MuJoCo에서 안 되고, 상자·원기둥 텍스처는 무시되며, 밀리는 물체도 고정된다(2절 끝 "MuJoCo에서는").

**정답 위치로만 보이는 함정**: `nav2_practice`에서 G4 → G1로 돌아올 때 로봇이 지도에 없는 낮은 턱 모서리에 걸렸다(2회 모두). 바퀴가 헛돌아 odom 오차가 143 cm까지 쌓였고 AMCL도 73 cm 어긋났는데, Nav2는 G1을 "성공"으로 보고했다. 정답 위치는 G1에서 약 77 cm 떨어져 있었다. 그 전 8.2 m 구간의 오차는 odom 0.1~2.7 cm, AMCL 0.3~23 cm였다.

**물리 설정 바꾸기**: 공식 `config/mujoco.yaml`을 복사해 고친 뒤 `physics_config:=<경로>`로 넘긴다(일부만이 아니라 파일 전체를 바꾼다). 예: `control.max_linear`를 0.25 → 0.10으로 바꾸면 0.25 m/s 명령에도 실제 속도가 0.099 m/s였다.

---

## 4. LED·램프·LCD 표정 — 가상 출력 장치 패널

실기의 출력 장치 서버(`led_server`, `pinky_lamp_control`, `emotion_server`)는 라즈베리파이 하드웨어를 직접 켜기 때문에 PC에서는 뜨지 않는다. 이 패키지의 `sim_devices`가 **같은 서비스 이름·타입·응답 문구**로 대신 응답하고, 결과를 패널 창에 보여 준다. launch에 기본으로 들어 있다(`devices:=false`로 끌 수 있다).

| 서비스 | 실기 | 시뮬레이션 |
|---|---|---|
| `/set_led`, `/set_brightness` | 로봇 LED | 패널의 LED 줄(번호 0~7) |
| `/set_lamp` | 로봇 램프 | 패널 램프 **+ Gazebo 로봇의 램프 색**(`gz_gui:=true`일 때 보임) |
| `/set_emotion` | 로봇 LCD | 패널의 LCD(실기와 같은 표정 GIF) |

공식 README의 명령을 그대로 보내면 된다.

```bash
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'fill', r: 255, g: 0, b: 0}"
ros2 service call /set_led pinky_interfaces/srv/SetLed "{command: 'set_pixel', pixels: [4, 5, 6, 7], r: 0, g: 0, b: 255}"
ros2 service call /set_brightness pinky_interfaces/srv/SetBrightness "{brightness: 50}"
ros2 service call /set_lamp pinky_interfaces/srv/SetLamp "{color: {r: 1.0, g: 0.0, b: 1.0, a: 1.0}, mode: 2, time: 300}"
ros2 service call /set_emotion pinky_interfaces/srv/Emotion "{emotion: 'angry'}"
```

램프 `mode`는 실기 `main_node.cpp`와 같다: `0` 끄기, `1` 켜기, `2` 깜빡임(`time` ms 꺼짐 → `time` ms 켜짐), `3` 디밍(`time` ms 동안 밝아졌다 어두워짐). 색은 0.0~1.0이다.

잘못된 호출도 실기처럼 실패로 응답한다(예: 8번 이후 픽셀 → `success: false`). 패널 아래쪽에 최근 호출 8개가 OK/ERR로 남는다.

> **시뮬레이션이 실기와 다를 수 있는 점**
> - LED 개수(기본 8)와 밝기 범위(기본 0~100)는 실기 라이브러리 `pinkylib`가 공개돼 있지 않아 추정값이다. 다르면 `led_count:=` 인자나 `max_brightness` 파라미터로 맞춘다.
> - 공식 Gazebo 램프 플러그인은 world 이름이 `pinky_factory`일 때만 초록색으로 숨쉬듯 깜빡인다. 이 패키지의 world에서는 동작하지 않으므로 `/set_lamp` 색과 섞이지 않는다.
> - 패널 한글이 네모로 보이면 WSL은 `setup_env.sh --windows-fonts`, 네이티브는 `sudo apt install fonts-noto-cjk`. 창 제목은 일부러 영문(`Pinky Virtual Devices`)이다.
> - 패널·Gazebo 램프가 처음에 **흰색으로 천천히 밝아졌다 어두워지는 것**은 정상이다. 실기 `pinky_lamp_control`의 시작 상태(흰색 디밍 1초)를 그대로 옮겼다. `/set_lamp`를 보내면 바뀐다.

---

## 5. 실물 Pinky에 그대로 쓰기

```bash
# Pinky에서
ros2 launch pinky_bringup bringup_robot.launch.xml
# PC에서 (같은 ROS_DOMAIN_ID)
ros2 run pinky_sim_examples command_driver
ros2 run pinky_sim_examples send "forward 0.3"
```

실물에서는 `use_sim_time`을 켜지 않는다. 처음에는 `speed 0.05`로 천천히 시험한다.

---

## 6. WSL에서 실행할 때

시뮬레이션 코드와 파라미터는 WSL과 네이티브 Ubuntu에서 같다. 차이는 실행 환경에서 나고, `setup_env.sh`가 아래 항목을 점검한다.

| 항목 | WSL2 | 네이티브 Ubuntu | 조치 |
|---|---|---|---|
| 실물 Pinky 연결 | 기본 NAT 네트워크면 로봇 토픽이 안 보일 수 있음 | 같은 Wi-Fi면 연결 | Windows `%UserProfile%\.wslconfig`에 `[wsl2]` / `networkingMode=mirrored` 넣고 `wsl --shutdown`. 방화벽 인바운드 허용이 필요할 수 있음 |
| 시뮬레이션 속도 | 실제 시간보다 느리게 돌 수 있음(측정 약 0.67배) | 보통 1배 가까이 | 주행 판단은 시뮬레이션 시각 기준이라 조치 불필요. `command_driver`가 시작할 때 `/scan` 주기를 재서 알려 준다 |
| 화면 그리기 | WSLg 경유 | GPU 드라이버 | Gazebo 창이 검거나 멈추면 `setup_env.sh --software-gl` |
| 한글 폰트 | 없을 수 있음(패널 한글이 네모) | 데스크톱 설치판은 보통 있음 | WSL: `setup_env.sh --windows-fonts` / 공통: `sudo apt install fonts-noto-cjk` |
| 작업 폴더 | `/mnt/c/...`에 두면 빌드가 매우 느림 | 해당 없음 | `~/pinky_pro` 처럼 리눅스 홈에 둔다 |

`command_driver` 시작 로그 읽는 법:

```
/scan 주기: 센서 시각 10.0 Hz, 실제 시간 6.7 Hz (설정 10 Hz, 시뮬레이션 속도 약 0.67배)
```

- **센서 시각** 값이 설정(10 Hz)보다 많이 낮으면 라이다가 늦게 나오는 것이다. 정지가 늦어질 수 있으니 `runtime.yaml`의 `stop_distance`를 늘리거나 속도를 낮춘다(이때 경고로 표시된다).
- **실제 시간** 값만 낮으면 PC가 시뮬레이션을 실제보다 느리게 돌리는 것이다. 로봇이 화면에서 느리게 움직일 뿐, 거리·정지 판단은 그대로다.

## 7. 파일 구성

| 경로 | 내용 |
|---|---|
| `layouts/twin_arena.yaml` | 기본 경기장 레이아웃 (+ 실습용 장애물 2개) |
| `layouts/my_room.yaml` | 연습용 2 × 2 m 방 |
| `worlds/*.world` | `make_world`로 생성한 결과 (직접 고치지 말고 YAML을 고칠 것) |
| `pinky_sim_examples/world_builder.py` | YAML → SDF 변환 (`make_world`) |
| `pinky_sim_examples/spawn_object.py` | 실행 중 물체 추가·삭제 |
| `pinky_sim_examples/commands.py` | 명령 해석·각도 계산 (ROS 의존 없음, 단위 테스트 대상) |
| `pinky_sim_examples/command_driver.py` | 명령 주행 노드 |
| `pinky_sim_examples/send_command.py` | 명령 보내고 결과 보기 (`send`) |
| `pinky_sim_examples/devices.py` | LED·램프·표정 동작 규칙 (ROS 의존 없음, 단위 테스트 대상) |
| `pinky_sim_examples/sim_devices.py` | 가상 출력 장치 서비스 + 패널 창 |
| `launch/twin_arena.launch.py` | 공식 `launch_sim` 내용을 옮겨 Gazebo 서버/창 분리(`gz_gui`) + 환경 설정 자동 로드 |
| `urdf/pinky_sim.urdf.xacro` | 공식 로봇 + 초음파 + (옵션) D455. 공식 URDF는 고치지 않음 |
| `config/sim_bridge.yaml` | 공식 브리지 + IMU(`imu_raw`) + 초음파 원본(`us_scan`) |
| `config/d455_bridge.yaml` | D455 영상·깊이·포인트클라우드 브리지 |
| `layouts/scan_room.yaml` | D455 스캔 실습용 정사각형 방 |
| `pinky_sim_examples/layout_markers.py` | 레이아웃 물체를 RViz 마커로 |
| `pinky_sim_examples/us_range.py`, `sensors.py` | 초음파 원본 → 실기와 같은 `us_sensor/range` |
| `docs/pinky_sim_guide.html` | 전체 실습 가이드(단독 HTML, 그림·실행 결과 포함) |
| `layouts/nav2_practice.yaml`, `maps/nav2_practice.*` | Nav2 실습장 맵과 미리 계산한 지도 |
| `pinky_sim_examples/map_builder.py` | 레이아웃 → Nav2 지도(pgm + yaml) (`make_map`) |
| `pinky_sim_examples/nav2_patrol.py` | Nav2 목표 지점 순회 예제 (`nav2_patrol`) |
| `launch/mujoco_sim.launch.py` | MuJoCo로 이 패키지 world 실행 (물리 설정 교체 포함) |
| `pinky_sim_examples/pose_error.py` | 정답 위치와 odom·AMCL 오차 비교 (`pose_error`, MuJoCo 전용) |
| `scripts/setup_env.sh` | 실행 환경 판별·점검·설정 파일 생성 |
| `test/` | `python3 -m pytest test` |

### 파라미터 (`command_driver`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `linear_speed` | 0.10 | 직진 속도 m/s |
| `angular_speed` | 0.8 | 최대 회전 속도 rad/s |
| `stop_distance` | 0.20 | 라이다 기준 정지 거리 m. 속도를 올리면 같이 늘릴 것(0.2 m/s에서 약 4 cm 더 밀림) |
| `sector_half_width_deg` | 20 | 앞·뒤 감지 부채꼴 반폭 |
| `lidar_mount_yaw` | π | 라이다 장착 회전 |

## 8. 문제 해결

| 증상 | 확인 |
|---|---|
| 명령을 보냈는데 반응 없음 | `ros2 topic pub`에 `--keep-alive 1`을 붙였는지, 또는 `send` 사용 |
| `/scan`이 안 나옴 | world에 Sensors 플러그인이 있는지 (`make_world`가 만든 world에는 들어 있다) |
| 로봇이 벽을 뚫고 지나감 | 그 물체에 `<collision>`이 있는지 |
| 벽이 공중에 떠 있거나 반쯤 묻힘 | pose z = 높이/2 인지 |
| launch에서 '환경 설정이 없어 기본값으로' 경고 | `scripts/setup_env.sh` 실행 (0-1절) |
| 이전 실행이 남아 이상하게 동작 | `pkill -9 -f "gz sim|parameter_bridge|rviz|ros2.*launch"` 후 다시 실행 |
