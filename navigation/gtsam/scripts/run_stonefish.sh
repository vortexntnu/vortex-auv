#!/usr/bin/env bash
# Build and open the established Nautilus/GTSAM/DP workspace setup.
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "$script_dir/../../../../.." && pwd)"
session=gtsam_stonefish
restart=false
case "${1:-}" in
  --restart) restart=true ;;
  --help|-h)
    echo "Usage: $0 [--restart]"
    echo "Build gtsam_navigation, launch joystick + DP + Stonefish + Foxglove, and open logs."
    echo "An existing session is reused unless --restart is supplied."
    exit 0 ;;
  '') ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac

# Avoid accidentally loading the other ros2_ws overlay from the user's shell.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
cd "$workspace"
command -v tmux >/dev/null
command -v colcon >/dev/null
if [[ ! -f install/local_setup.bash ]]; then
  echo "Build the existing Nautilus/Stonefish workspace dependencies first." >&2
  exit 1
fi
source install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
unset PYTHONNOUSERSITE

cmake_args=()
gtsam_config="$script_dir/../.deps/install/lib/cmake/GTSAM"
if [[ -f "$gtsam_config/GTSAMConfig.cmake" ]]; then
  cmake_args=(--cmake-args "-DGTSAM_DIR=$gtsam_config")
fi
PYTHONNOUSERSITE=1 colcon build --packages-select gtsam_navigation "${cmake_args[@]}"
source install/local_setup.bash
for package in stonefish_ros2 stonefish_sim auv_setup dp_adapt_backs_controller_quat \
  joystick_interface_auv operation_mode_manager thrust_allocator_auv joy foxglove_bridge; do
  ros2 pkg prefix "$package" >/dev/null
done

if $restart && tmux has-session -t "$session" 2>/dev/null; then
  # Stop only the identified launch in our named session; never broad pkill.
  python3 - "$session" <<'PY'
import os
import signal
import subprocess
import sys
import time

session = sys.argv[1]
result = subprocess.run(
    ['tmux', 'list-panes', '-t', session + ':simulation', '-F', '#{pane_pid}'],
    capture_output=True, text=True,
)
if result.returncode == 0:
    pid = int(result.stdout.strip())
    with open(f'/proc/{pid}/cmdline', 'rb') as stream:
        command = stream.read()
    if b'ros2' not in command or b'stonefish.launch.py' not in command:
        raise SystemExit('Unexpected process in simulation pane; refusing to stop it.')
    os.kill(pid, signal.SIGINT)
    for _ in range(400):
        try:
            with open(f'/proc/{pid}/stat') as stream:
                state = stream.read().rsplit(')', 1)[1].split()[0]
            if state == 'Z':
                break
        except FileNotFoundError:
            break
        time.sleep(0.1)
    else:
        raise SystemExit('Launch did not shut down; refusing to start a duplicate.')
if subprocess.run(['tmux', 'has-session', '-t', session], capture_output=True).returncode == 0:
    subprocess.run(['tmux', 'kill-session', '-t', session], check=True)
PY
fi

if ! tmux has-session -t "$session" 2>/dev/null; then
  # Fail clearly if another stack owns the port; leave that stack untouched.
  python3 - <<'PY'
import socket
with socket.socket() as listener:
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        listener.bind(('0.0.0.0', 8765))
    except OSError as error:
        raise SystemExit(f'Port 8765 is occupied. Stop the other bridge first: {error}')
PY
  printf -v setup 'source /opt/ros/humble/setup.bash; source %q/install/local_setup.bash; export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; ' "$workspace"
  printf -v launch_command 'bash --noprofile --norc -c %q' "${setup}exec ros2 launch gtsam_navigation stonefish.launch.py input:=joystick foxglove:=true dvl_max_tilt_deg:=25.0"
  tmux new-session -d -s "$session" -n simulation -c "$workspace" "$launch_command"
else
  echo "Reusing existing $session session. Use --restart for a fresh simulation."
fi

if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null; then
  gnome-terminal --title='Nautilus stack logs' -- tmux attach-session -t "$session"
  gnome-terminal --title='Nautilus controller diagnostics' -- python3 "$script_dir/monitor_stonefish.py"
fi
if ! pgrep -f '^/opt/Foxglove/foxglove-studio( |$)' >/dev/null && [[ -x /opt/Foxglove/foxglove-studio ]]; then
  mkdir -p "$workspace/log"
  nohup /opt/Foxglove/foxglove-studio >"$workspace/log/foxglove-desktop.log" 2>&1 </dev/null &
fi
cat <<'TEXT'
Foxglove: ws://localhost:8765 — select the saved "gtsam estimation" layout.
Logs: tmux attach -t gtsam_stonefish
Wait for GTSAM initialization. Tap B briefly to release the killswitch.
A: manual thrust. Lift off the floor manually, then Y: DP reference control.
Left stick: forward/back/sideways. Right stick: yaw/pitch. LB/RB: roll.
Triggers: vertical movement. X/autonomous is not supported in this setup.
Keep Stonefish open and running continuously; closing it stops sensor data.
TEXT
