#!/usr/bin/env bash
# Pinky 시뮬레이션 실행 환경 설정 (clone 후 한 번, 환경이 바뀌면 다시 실행)
#
#   bash scripts/setup_env.sh                 # 자동 판별 (WSL / 네이티브 Ubuntu)
#   bash scripts/setup_env.sh --profile wsl   # 강제 지정
#   bash scripts/setup_env.sh --domain 7      # ROS_DOMAIN_ID 지정 (실물 Pinky와 같게)
#   bash scripts/setup_env.sh --software-gl   # Gazebo 창이 검거나 멈출 때
#   bash scripts/setup_env.sh --bashrc        # ~/.bashrc 에 env.sh source 줄 추가
#   bash scripts/setup_env.sh --windows-fonts # (WSL) Windows 맑은 고딕을 사용자 폰트로 연결(sudo 불필요)
#
# 만드는 파일 (git 저장소 밖, 사람마다 다른 값)
#   ~/.config/pinky_sim_examples/env.sh        환경 변수 (source 해서 사용)
#   ~/.config/pinky_sim_examples/runtime.yaml  노드 파라미터 (launch가 자동으로 읽음)
#
# sudo 나 Windows 설정이 필요한 항목은 바꾸지 않고, 실행할 명령만 알려 준다.
set -u

PROFILE="" DOMAIN="${ROS_DOMAIN_ID:-0}" SOFT_GL=0 BASHRC=0 WIN_FONTS=0
OUT_DIR="${PINKY_SIM_CONFIG_DIR:-$HOME/.config/pinky_sim_examples}"
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) PROFILE="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --software-gl) SOFT_GL=1; shift ;;
    --bashrc) BASHRC=1; shift ;;
    --windows-fonts) WIN_FONTS=1; shift ;;
    -h|--help) sed -n 2,16p "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $1 (--help 참고)"; exit 2 ;;
  esac
done

WARN=0
ok()   { printf '  [OK]   %s\n' "$1"; }
warn() { printf '  [확인] %s\n' "$1"; WARN=$((WARN + 1)); }
todo() { printf '         -> %s\n' "$1"; }

# ---------- 1. 환경 판별 ----------
if [ -z "$PROFILE" ]; then
  if grep -qi microsoft /proc/version 2>/dev/null; then PROFILE=wsl; else PROFILE=native; fi
fi
case "$PROFILE" in wsl|native) ;; *) echo "--profile 은 wsl 또는 native"; exit 2 ;; esac
if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]] || [ "$DOMAIN" -gt 232 ]; then
  echo "--domain 은 0~232 정수: $DOMAIN"; exit 2
fi

# 환경별 노드 파라미터. 근거는 README 'WSL에서 실행할 때' 절 참고
# 2026-11-18 측정: WSL에서도 /scan 은 센서(시뮬레이션) 시각 기준 10 Hz로 정상이었다.
# 실제 시간 기준으로만 느렸는데(시뮬레이션 0.67배속), 주행 제어도 시뮬레이션 시각으로 돌아서 영향이 없다.
# 그래서 지금은 두 환경 값이 같다. 차이가 확인되면 이 블록만 고치면 된다.
if [ "$PROFILE" = wsl ]; then
  STOP_DISTANCE=0.20
  SCAN_WARN_RATIO=0.8
else
  STOP_DISTANCE=0.20
  SCAN_WARN_RATIO=0.8
fi

echo "== Pinky 시뮬레이션 환경 설정: profile=$PROFILE, ROS_DOMAIN_ID=$DOMAIN"

# ---------- 2. 점검 ----------
echo "-- 점검"
if [ -f /opt/ros/jazzy/setup.bash ]; then ok "ROS 2 Jazzy 설치됨"
else warn "ROS 2 Jazzy(/opt/ros/jazzy)가 없습니다. 공식 repo는 Ubuntu 24.04 + Jazzy 기준"; fi

PKG_DIR="$(cd "$(dirname "$0")/.." && pwd -P)"
case "$PKG_DIR" in
  /mnt/*) warn "패키지가 Windows 드라이브($PKG_DIR) 아래입니다. 빌드가 매우 느립니다"
          todo "리눅스 홈으로 옮기기: mkdir -p ~/pinky_pro/src && cp -r <폴더> ~/pinky_pro/src/" ;;
  *) ok "작업 폴더가 리눅스 파일시스템에 있음" ;;
esac

WIN_FONT_DIR="${PINKY_WIN_FONT_DIR:-/mnt/c/Windows/Fonts}"
USER_FONT_DIR="$HOME/.local/share/fonts"
if [ "$WIN_FONTS" = 1 ]; then
  if [ "$PROFILE" = wsl ] && ls "$WIN_FONT_DIR"/malgun*.ttf >/dev/null 2>&1; then
    mkdir -p "$USER_FONT_DIR"
    ln -sf "$WIN_FONT_DIR"/malgun*.ttf "$USER_FONT_DIR"/
    command -v fc-cache >/dev/null && fc-cache -f "$USER_FONT_DIR" >/dev/null 2>&1
    ok "Windows 맑은 고딕을 $USER_FONT_DIR 에 연결 (되돌리기: rm $USER_FONT_DIR/malgun*.ttf)"
  else
    warn "--windows-fonts 는 WSL에서 $WIN_FONT_DIR/malgun.ttf 가 있을 때만 동작합니다"
  fi
fi
if command -v fc-list >/dev/null && [ -n "$(fc-list :lang=ko 2>/dev/null | head -1)" ]; then
  ok "한글 폰트 있음 (패널 표시용)"
else
  warn "한글 폰트가 없습니다. 가상 장치 패널의 한글이 네모로 보입니다"
  if [ "$PROFILE" = wsl ]; then todo "bash $0 --windows-fonts   (sudo 불필요, Windows 맑은 고딕 사용)"; fi
  todo "sudo apt install fonts-noto-cjk"
fi

if python3 -c 'import tkinter, PIL.ImageTk' 2>/dev/null; then ok "패널용 tkinter·Pillow 있음"
else warn "tkinter 또는 PIL.ImageTk 가 없습니다"; todo "sudo apt install python3-tk python3-pil.imagetk"; fi

if [ "$PROFILE" = wsl ]; then
  MODE="$(wslinfo --networking-mode 2>/dev/null || echo unknown)"
  if [ "$MODE" = mirrored ]; then
    ok "WSL 네트워크 mirrored (실물 Pinky와 같은 네트워크로 보임)"
  else
    warn "WSL 네트워크가 '$MODE' 입니다. 시뮬레이션은 되지만 실물 Pinky 토픽이 안 보일 수 있습니다"
    todo "Windows의 %UserProfile%\\.wslconfig 에 다음을 넣고 PowerShell에서 wsl --shutdown 후 재시작"
    todo "  [wsl2]"
    todo "  networkingMode=mirrored"
    todo "Windows 방화벽에서 WSL 인바운드 허용 필요할 수 있음 (README 'WSL에서 실행할 때')"
  fi
  if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then ok "WSLg 화면 사용 가능"
  else warn "DISPLAY 가 없습니다. Gazebo·패널 창이 뜨지 않습니다"; todo "PowerShell: wsl --update"; fi
fi

# ---------- 3. 파일 생성 ----------
echo "-- 설정 파일 생성: $OUT_DIR"
mkdir -p "$OUT_DIR"
{
  echo "# setup_env.sh 가 만든 파일 ($(date '+%F %T')). 직접 고쳐도 되지만 다시 실행하면 덮어씁니다."
  echo "export PINKY_SIM_PROFILE=$PROFILE"
  echo "export PINKY_SIM_CONFIG_DIR=$OUT_DIR"
  echo "export ROS_DOMAIN_ID=$DOMAIN"
  if [ "$SOFT_GL" = 1 ]; then echo "export LIBGL_ALWAYS_SOFTWARE=1   # --software-gl"; fi
} > "$OUT_DIR/env.sh"

cat > "$OUT_DIR/runtime.yaml" <<EOF
# setup_env.sh 가 만든 파일 (profile: $PROFILE). launch 가 자동으로 읽습니다.
# 값을 바꾸고 싶으면 이 파일을 고치거나 setup_env.sh 를 다시 실행하세요.
command_driver:
  ros__parameters:
    stop_distance: $STOP_DISTANCE
    expected_scan_hz: 10.0          # pinky_gz.urdf.xacro gpu_lidar update_rate
    scan_rate_warn_ratio: $SCAN_WARN_RATIO
EOF
ok "env.sh, runtime.yaml"

if [ "$BASHRC" = 1 ]; then
  LINE="[ -f $OUT_DIR/env.sh ] && source $OUT_DIR/env.sh   # pinky_sim_examples"
  if grep -qF "$OUT_DIR/env.sh" "$HOME/.bashrc" 2>/dev/null; then ok "~/.bashrc 에 이미 있음"
  else echo "$LINE" >> "$HOME/.bashrc"; ok "~/.bashrc 에 source 줄 추가"; fi
fi

echo
echo "다음 단계:"
echo "  source $OUT_DIR/env.sh      (새 터미널마다. --bashrc 로 자동화 가능)"
echo "  ros2 launch pinky_sim_examples twin_arena.launch.py"
[ "$WARN" -gt 0 ] && echo "확인할 항목 ${WARN}개가 있습니다(위 [확인] 줄)."
exit 0
