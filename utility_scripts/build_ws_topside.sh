#!/bin/bash
set -e

WS_DIR="$HOME/vscopium/ros2_ws"
SRC_DIR="$WS_DIR/src"
GITHUB_ORG="https://github.com/vortexntnu"

# Repos to clone
REPOS=(
  "stonefish_ros2"
  "vortex-auv"
  "vortex-msgs"
  "vortex-stonefish-interface"
  "vortex-stonefish-sim"
  "vortex-utils"
  "vortex-ci"
)

# ---- Install dependencies script ----

# python3 -m pip install --upgrade pip
# install numpy, pynput, joy, wheel
# install ros-humble-xacro, ros-humble-joy
# run casadi install script

# ------------ Clone missing repos ------------
mkdir -p "$SRC_DIR"

for repo in "${REPOS[@]}"; do
  if [ -d "$SRC_DIR/$repo" ]; then
    echo "[SKIP] $SRC_DIR/$repo already exists"
  else
    echo "[CLONE] $GITHUB_ORG/$repo.git -> $SRC_DIR/$repo"
    git clone "$GITHUB_ORG/$repo.git" "$SRC_DIR/$repo"
  fi
done

# ---------- Build ----------
cd "$WS_DIR"

# Default packages to build
DEFAULT_PKGS=(
  vortex_msgs
  vortex_utility_nodes
  vortex_utils
  vortex_utils_ros
  stonefish_ros2
  vortex_sim_interface
  stonefish_sim
  stonefish_sim_interface
  thrust_allocator_auv
  thruster_interface_auv
  dp_adapt_backs_controller
  pid_controller_dp
  reference_filter_dp
  keyboard_joy
  joystick_interface_auv
  auv_setup
  operation_mode_manager
  los_guidance
)

# Use arguments if provided, otherwise use defaults
if [ $# -gt 0 ]; then
  PKGS=("$@")
else
  PKGS=("${DEFAULT_PKGS[@]}")
fi

BUILD_FAILED=0
echo "[BUILD] Building packages: ${PKGS[*]}"
colcon build \
  --packages-up-to "${PKGS[@]}" \
  --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ||
  BUILD_FAILED=1

# ---------- Handle result ----------
if [ $BUILD_FAILED -eq 1 ]; then
  echo ""
  echo "    ╔══════════════════════════════════════════════════╗    "
  echo "    ║                   BUILD FAILED                   ║    "
  echo "    ╚══════════════════════════════════════════════════╝    "
  echo ""
  echo "⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⣶⡖⢀⠀⠒⠒⠒⠒⠒⠒⠒⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀"
  echo "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠛⠙⠉⠉⠉⠉⠉⠉⠉⠙⠃⠀⠀⠠⠄⠂⠐⠀⠀⠀⠀⠀⠀⠀⠉⠁⠀⠀⠀⢸⠀⠀⠀⠀⠀"
  echo "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀"
  echo "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀⠀⠀⠀⠀⡀⣀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀"
  echo "⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀⠀⠀⢀⣶⢿⣵⣮⣞⣶⣆⡶⣴⣀⠀⠀⠀⠀⠀⠀⠡⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⠉⠉⠁⠂⠐⠂⡐⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠓⢀⡖⠀⢠⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣭⢚⠤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠅"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⣾⠃⣴⣿⣿⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⡎⣧⢣⡅⢣⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠐⠨⣿⣿⣿⣿⣿⣿⣿⠿⠋⠀⠉⢠⢿⣟⣯⣿⣾⣿⣿⣿⣿⣿⣿⣿⣿⡿⣜⠮⡜⣌⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠈"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠚⠿⠿⠿⠿⠿⠏⠀⠂⣴⣲⢠⣛⣾⣿⣿⣿⣿⣿⢿⣿⣿⣿⣿⣿⣿⣯⡟⡴⢂⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠔"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠈⠷⡙⢰⣉⡴⣀⡉⠉⠋⠙⢉⣿⡟⠛⠛⠋⠛⠙⡉⠘⠁⠀⢀⣞⠀⠀⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠡⠈"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⡀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠃⠀⠉⠘⠁⠀⢀⣠⣾⣧⡀⠀⠀⠐⠃⠂⠁⠀⠀⠠⠈⠄⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠂"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠠⠀⠀⠀⠀⠀⠀⠀⢀⣋⢶⣠⣤⣤⡶⢮⣿⣿⣿⣷⠄⢦⣄⣀⡀⢄⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠁⠒⣀⠀⠀⠀⠀⠀⠀⠜⣿⡿⣿⡟⣡⡿⢿⣿⣿⢿⡒⢌⠻⣯⠗⡎⠄⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠀⠒⠤⣄⣀⢀⡀⠀⠀⢙⢣⡽⡅⠀⠀⠀⠀⠀⠀⢈⡖⠁⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡶⡲⡒⡒⣴⡶⣒⣖⢶⣶"
  echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠈⠄⡁⠢⠐⠠⢌⠁⠁⠀⠱⣈⠎⢙⡳⣍⠾⠶⡲⠦⣐⠎⠲⠀⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠓⠔⠥⠇⠛⠆⠋⠭⠚⠛"
  echo "⠀⠀⠐⠀⠀⠀⠀⢀⠀⠀⠂⢌⡐⢠⠒⣤⣁⠎⠅⣈⡀⠀⠀⠀⠑⢪⠀⡈⣈⣁⡀⠁⣠⡁⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⢀⣄⣐⣠⡈⠤⡁⠆⡑⣀⠉⠆⠉⢂⡙⢀⣂⣤⠚⣵⢯⡄⠀⠀⠀⠀⠡⠀⠀⠉⠘⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⠈⣨⣧⣶⣟⠷⢛⣡⢤⡲⡜⢮⠌⣍⢿⣦⠐⣈⢷⡌⠞⡜⢆⠀⠀⠀⠀⠡⣀⢄⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⣾⣿⠟⣴⣞⣯⠻⣜⣣⢟⣹⢂⢯⡘⢎⡞⣦⠀⢂⠙⢦⠉⢄⠊⢀⠀⠀⠀⠀⠊⠐⠁⠈⠀⠀⠀⠀⠀⠀⠀⠀⡀⢀⠠⠀⠀⡣⢤⡉⢄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
  echo "⣿⡧⣏⢳⡬⢷⡙⢆⡳⢎⡳⢜⡢⢝⡲⢸⢥⠣⢌⠢⣤⣉⠢⠄⡄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⢆⡱⠌⢀⠐⢢⢍⠢⢈⠄⡀⠀⠀⠀⠀⠀⠀⠀"
  echo "⡟⣠⠻⣄⠻⣜⢻⡘⢣⡛⠼⣀⠻⡘⡤⢛⡄⢿⡀⢧⣛⢧⢇⡄⠀⠠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⠀⠀⢠⡘⠄⡤⢀⠄⡀⠘⡄⠇⡘⠠⣀⠀⠀⠀⠀⠀⠀⠀"
  echo "⠐⣡⠛⣌⠳⣌⠳⣌⠣⡜⡱⢌⢣⠱⢌⠣⡜⢢⡑⢎⡝⣊⠞⡼⣉⠖⡠⢐⠀⠀⠀⠀⠀⠀⠀⠀⠀⢐⠀⡘⠤⣌⠳⠀⡌⠎⡄⠀⠘⡀⣁⠒⢠⠂⠀⠀⠀⠀⠀⠀"
  echo "⠠⢡⠉⡄⠣⢌⠳⡄⡓⢬⡐⠎⣄⠣⢊⠱⡘⡄⢙⢢⠹⣤⢋⡴⠡⢎⠱⡈⠆⠀⢀⠀⠀⠀⠀⢀⠀⡜⠠⣘⠲⢌⡱⠠⡑⢌⠰⡡⠀⠐⢠⠘⠠⢀⠃⠀⠀⠀⠀⠀"
  echo "⢡⠂⠥⠐⡀⢈⠳⣐⡉⠆⡜⠒⡄⢣⠉⢆⠱⡐⠈⢂⠓⡌⢣⠘⡡⢊⠐⡘⠄⠀⠂⡀⠂⠌⣠⡔⢂⠩⠐⢤⢋⠆⡀⠂⠔⡈⠆⠥⠁⠀⠠⠈⡐⠨⢘⠀⠀⠀⠀⠀"
  echo "⢀⠣⢈⠱⡐⠀⡇⠢⢅⡍⠰⡉⡔⢡⠊⡄⢣⠘⡀⠀⢂⠐⠠⢁⠐⠀⢂⡉⠆⠀⠡⢐⣹⣾⣿⣿⡆⡣⢘⢢⢉⠂⠠⢁⡘⠤⣉⠒⠀⢂⠀⠀⠤⠑⢂⠂⠀⠀⢀⠀"
  echo "⡀⢃⠆⠐⡈⠡⢌⡑⠢⢌⠱⡐⢌⠢⡑⢌⠂⡅⢂⠁⡀⠂⢁⠠⠈⢀⠀⡜⠀⢈⠐⢿⣿⣿⣿⣿⡇⡅⡌⠆⠌⠀⠐⠠⡐⢢⠐⡀⠈⢀⠀⠀⠠⢁⠂⠌⠀⢀⢰⣤"
  echo "⡘⠆⠌⠀⠠⢁⠢⠌⡁⠎⢠⠑⡈⠆⡑⢂⠥⢘⠠⢂⠀⠄⠂⠀⡀⠂⠠⢌⠀⣢⡍⢎⡻⣿⣿⣿⡇⣼⣿⣎⠀⠀⢂⠡⠐⢂⠡⠀⢀⠂⠌⠀⠀⠠⠀⠌⠀⠀⠂⠹"
  echo ""
  echo ""
  exit 1
fi

# ---------- Merge compile_commands.json ----------
# Each package gets its own compile_commands.json under build/<pkg>/.
# Merge them into a single file at the workspace root for clangd / IDEs.
echo "[MERGE] Combining compile_commands.json files..."
python3 -c "
import json, glob, pathlib

ws = pathlib.Path('$WS_DIR')
combined = []
for f in sorted(ws.glob('build/*/compile_commands.json')):
    combined.extend(json.loads(f.read_text()))

out = ws / 'compile_commands.json'
out.write_text(json.dumps(combined, indent=2))
print(f'  -> Wrote {len(combined)} entries to {out}')
"

# ---------- Source ----------
source "$WS_DIR/install/setup.bash"

echo ""
echo "      ╔══════════════════════════════════════════════════╗"
echo "      ║                 BUILD SUCCESSFUL                 ║"
echo "      ╚══════════════════════════════════════════════════╝"
echo ""
echo "⠀⠀⢀⠀⣠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
echo "⢀⠀⣿⡂⢹⡇⠀⠀⣰⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
echo "⢸⡇⢸⣇⢸⣇⠀⢀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢾⠀⠀⣯⡀⡆⠀⠀"
echo "⢸⣷⢸⣇⣸⣇⠀⣾⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣠⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢳⣂⠀⣿⡄⢸⡀⣤"
echo "⢠⣿⣿⣿⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⣊⡝⠛⠙⠂⠄⠠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣦⣼⣷⣼⣁⠼"
echo "⢸⣿⣿⣿⣿⣿⣿⣀⢀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⡻⣥⢋⡔⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣿⣂⣜⣿⡟⢿⣿⣿⣄"
echo "⠈⣿⣿⣿⣿⣿⣿⣿⠿⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣷⢯⣿⣾⡔⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢪⣷⣿⢿⣿⣿"
echo "⠀⣿⣿⣟⢿⠿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣿⡟⠛⠉⡉⢸⡉⠁⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢢⣽⣗⣿⠇"
echo "⠀⣿⣿⣿⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠺⣿⡇⣤⡤⢔⡿⣇⠀⢦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⣿⣯⠀"
echo "⠘⡟⣛⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⡇⣿⣿⠗⡲⠏⠟⠿⠀⠈⠓⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⠍⠁⠁⠀"
echo "⠃⡜⡠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣼⣿⡟⢡⡿⠿⠷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣟⠒⠂⠂"
echo "⠐⢐⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⠸⣡⢶⣿⣟⡃⠀⠘⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⡇⠀⡀⠀"
echo "⢠⡏⠀⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡰⢨⠣⠉⠉⠋⠉⠀⠀⠀⠀⢈⠀⡂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⡿⠀⠀⠀⠀"
echo "⢺⡇⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣽⡿⢛⢭⠏⣢⠍⠈⠖⠀⠀⠒⣶⢦⡁⠂⠀⠀⠀⠀⠀⠯⠤⣤⣴⢶⣍⠝⣯⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⢌⣿⠱⠀⠀⠀⠀⠀"
echo "⣯⣯⠸⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠄⠀⠈⠀⠁⠀⠀⠀⠀⠀⠀⠀⠂⠀⠀⠏⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠧⠍⠶⠤⠈⣆⠀⠀⠀⠀⠀⠀⠀⣷⡻⠀⣼⠀⠀⠀"
echo "⣯⣨⡀⢀⡠⠤⣐⠤⣀⣰⠔⠊⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⠑⠐⠐⠢⠺⠥⡾⠉⡠⠀⠀⠀"
echo "⠋⠙⠈⠉⠉⠁⠈⠈⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
echo "⠓⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀"
echo "⠀⠀⠇⣣⡁⢶⣠⢀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⢶⠀⡶⣲⠀⣆⡒⣰⠒⢦⢰⠀⢰⡆⣴⠐⣶⠒⣐⣒⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣴⣺⣿⣿⣿⠛"
echo "⠀⠀⠑⢌⠻⣗⣔⠉⡅⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠞⠚⠃⠻⠴⠃⠦⠝⠘⠤⠎⠸⠤⠘⠧⠞⠀⠛⠀⠰⠤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⡟⣾⣿⣿⣿⠃⠀"
echo "⠀⠀⠀⠀⠉⠢⠁⠀⠀⠀⠀⢀⣤⣤⣤⣄⠀⠀⢠⣤⠀⠀⣤⣄⠀⠀⠀⣤⣤⠀⢠⣤⣤⣤⣤⣤⡄⢠⣤⣄⠀⠀⠀⠀⣤⣤⡄⠀⠀⠀⢠⣤⡄⠀⠀⠀⢘⡮⡝⣿⣿⡿⢆⠁⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⠏⠉⠉⢿⣷⠀⢸⣿⠀⠠⣿⣿⣧⡀⠀⣿⣿⠀⢸⣿⡏⠉⠉⠉⠁⢼⣿⣿⡄⠀⠀⢸⡿⣿⡇⠀⠀⢀⣿⢻⣷⠀⠀⠀⠞⡜⣹⣿⣿⡙⢆⠀⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⠀⢸⣿⠀⠐⣿⡯⢻⣷⡀⣿⣿⠀⢸⣿⣷⣶⣶⡆⠀⢺⣿⠹⣿⡀⢠⣿⠃⣿⡇⠀⠀⣾⡟⠀⢿⣧⠀⠀⠀⠠⢽⣿⣯⡙⠀⠀⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣿⡀⠀⠀⣠⣤⠀⢸⣿⠀⢈⣿⡧⠀⠹⣿⣿⣿⠀⢸⣿⡇⠀⠀⠀⠀⢸⣿⡄⢻⣧⣾⡏⢠⣿⡇⠀⣼⣿⣷⣶⣾⣿⣇⠀⠀⠀⠘⣿⢣⠜⠁⠀⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣶⣾⣿⠏⠀⢸⣿⠀⠀⣿⡷⠀⠀⠹⣿⣿⠀⢸⣿⣿⣿⣿⣿⡆⢸⣿⡆⠀⢿⡿⠀⢰⣿⡇⢀⣿⡏⠀⠀⠀⢹⣿⡀⠀⠀⠀⠀⠈⡆⠀⠀⠀"
echo "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠀⠀⠀⠈⠉⠀⠀⠉⠁⠀⠀⠀⠉⠉⠀⠈⠉⠉⠈⠉⠉⠁⠈⠉⠀⠀⠈⠁⠀⠀⠉⠁⠈⠉⠀⠀⠀⠀⠈⠉⠁⠐⡀⠀⠀⠀⠀⠀⠀⠀"
echo ""
echo ""
echo "           Remember to source your new changes bossman!          "
echo ""
