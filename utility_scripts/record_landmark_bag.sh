#!/bin/bash
# Record a bag for tuning landmark_server offline: the inputs it needs
# (detections, odometry, TF), the raw navigation sensors (to see where drift
# comes from), what the live landmark_server and guidance did, and optionally
# compressed camera images. One bag per test, named after it, with a notes
# file next to it.
#
# Usage: ./record_landmark_bag.sh <test> [OPTIONS]
#   <test>          What this run is: A-loops, B-hover, C-range, D-bias,
#                   mission, or any short name without spaces
#   --images        Also record compressed camera images (large)
#   --extra <regex> Also record topics matching this regex, e.g.
#                   '^/nautilus/front_camera$' (the simulator's raw image)
#   --dir <path>    Where to put the bags (default: ~/bags)
#   --ns <name>     Vehicle namespace (default: nautilus)
# Stop with Ctrl-C. The bag info is printed afterwards.

usage() {
    sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'
}

TEST=""
IMAGES="false"
EXTRA=""
DIR="$HOME/bags"
NS="nautilus"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --images) IMAGES="true"; shift ;;
        --extra)  EXTRA="$2"; shift 2 ;;
        --dir)    DIR="$2"; shift 2 ;;
        --ns)     NS="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "Unknown option: $1"; usage; exit 1 ;;
        *)  TEST="$1"; shift ;;
    esac
done
if [[ -z "$TEST" || "$TEST" =~ [[:space:]/] ]]; then
    echo "Give the test name (no spaces), e.g. A-loops"
    usage
    exit 1
fi

case "$TEST" in
    A*) HINT="Start in front of the gate, drive a 20-40 m loop, come back to the gate and look at it for 10 s. Repeat 3-5 times (one bag per loop or all in one)." ;;
    B*) HINT="Hold still for 2-3 min with an object (gate or board) in view." ;;
    C*) HINT="Stand still straight in front of one object at about 2, 4, 6 and 8 m, 30 s each. Say the distances in the notes." ;;
    D*) HINT="Look at the same object from several distances and angles (a half circle around it)." ;;
    *)  HINT="Write in the notes what the run does." ;;
esac

STAMP="$(date +%Y-%m-%d_%H%M%S)"
OUT="$DIR/${STAMP}_${TEST}"
mkdir -p "$DIR"

# Inputs of landmark_server, raw navigation sensors, what the live stack did.
REGEX="^/(tf|tf_static)$"
REGEX="$REGEX|^/$NS/(landmarks|odom|pose|twist|pressure|magnetometer|temperature)$"
REGEX="$REGEX|^/$NS/(dvl|imu)/.*"
REGEX="$REGEX|^/$NS/landmark_server/.*"
REGEX="$REGEX|^/$NS/(guidance|reference_filter)/.*|^/$NS/(reference_pose|waypoint|operation_mode|killswitch)$"
if [[ "$IMAGES" == "true" ]]; then
    REGEX="$REGEX|^/$NS/.*(camera|image).*/compressed$"
fi
if [[ -n "$EXTRA" ]]; then
    REGEX="$REGEX|$EXTRA"
fi

cat > "${OUT}_notes.md" <<EOF
# ${STAMP} ${TEST}

Test: ${TEST}
What to do: ${HINT}

## Notes (fill in)
- Pool / date:
- Objects and their measured positions (tape), distances between them:
- Pool floor depth:
- What the run did (times, distances):
- Anything odd (hit a wall, DVL dropout, detector restarted, killswitch):
EOF

echo "Recording test '$TEST' to $OUT"
echo "What to do: $HINT"
echo "Notes file: ${OUT}_notes.md"
echo "Stop with Ctrl-C."
echo

# mcap if the plugin is installed (ros-humble-rosbag2-storage-mcap), else
# the default sqlite3.
STORAGE=()
if ros2 bag record --help 2>/dev/null | grep -q "mcap"; then
    STORAGE=(-s mcap)
fi
ros2 bag record "${STORAGE[@]}" -o "$OUT" -e "$REGEX"

echo
ros2 bag info "$OUT" 2>/dev/null | sed -n '1,60p'
echo
echo "Check that /$NS/landmarks, /$NS/odom and /tf have messages, then fill in ${OUT}_notes.md."
