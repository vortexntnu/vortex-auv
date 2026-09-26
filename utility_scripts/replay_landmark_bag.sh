#!/bin/bash
# Replay a bag into one or more landmark_servers with different parameters,
# to tune offline. Only the inputs are played (detections, odometry, TF), so
# the servers here do not collide with what the live server recorded.
# Each server runs in its own namespace, /tune_<label>, and publishes its map
# on /tune_<label>/landmark_server/object_map (markers, graph path, stats
# likewise).
#
# Usage: ./replay_landmark_bag.sh <bag> [OPTIONS] [<label> "<-p overrides>"]...
#   <bag>             The bag directory
#   --env <sim|pool>  Environment config (default: pool)
#   --rate <r>        Playback rate (default: 1.0). Keep 1.0 for tuning:
#                     the server ticks on wall time (200 ms), so at 2.0 its
#                     N/M windows cover twice as much bag time
#   --ns <name>       Vehicle namespace in the bag (default: nautilus)
#   <label> "..."     A server named <label> with these parameter overrides,
#                     e.g.  base ""  nograph "-p graph.enable:=false"
#                     Default: one server "base" with no overrides.
# After playback the servers keep their final maps until Ctrl-C.

usage() {
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

BAG=""
ENV_NAME="pool"
RATE="1.0"
NS="nautilus"
LABELS=()
OVERRIDES=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --env)  ENV_NAME="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --ns)   NS="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "Unknown option: $1"; usage; exit 1 ;;
        *)
            if [[ -z "$BAG" ]]; then
                BAG="$1"; shift
            else
                LABELS+=("$1"); OVERRIDES+=("${2:-}"); shift 2 || shift
            fi
            ;;
    esac
done
if [[ -z "$BAG" || ! -d "$BAG" ]]; then
    echo "Give the bag directory"
    usage
    exit 1
fi
if [[ ${#LABELS[@]} -eq 0 ]]; then
    LABELS=("base"); OVERRIDES=("")
fi

WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
C="$WS/install/landmark_server/share/landmark_server/config"
D="$WS/install/auv_setup/share/auv_setup/config/robots/${NS}.yaml"
if [[ ! -f "$C/${ENV_NAME}.yaml" ]]; then
    echo "No $C/${ENV_NAME}.yaml; build landmark_server first."
    exit 1
fi

cleanup() {
    for label in "${LABELS[@]}"; do
        pkill -INT -f "__ns:=/tune_${label} " 2>/dev/null
    done
    sleep 2
    for label in "${LABELS[@]}"; do
        pkill -KILL -f "__ns:=/tune_${label} " 2>/dev/null
    done
}
trap cleanup EXIT

for i in "${!LABELS[@]}"; do
    label="${LABELS[$i]}"
    # shellcheck disable=SC2086
    ros2 run landmark_server landmark_server_node --ros-args \
        -r __ns:=/tune_"$label" -r __node:=landmark_server_node \
        --params-file "$C/landmark_server_config.yaml" \
        --params-file "$C/${ENV_NAME}.yaml" --params-file "$D" \
        -p use_sim_time:=true \
        -p topics.landmarks:=/"$NS"/landmarks -p topics.odom:=/"$NS"/odom \
        -p course_frame.publish_tf:=false \
        ${OVERRIDES[$i]} > "/tmp/replay_${label}.log" 2>&1 &
    echo "server $label -> /tune_$label  (log /tmp/replay_${label}.log)  ${OVERRIDES[$i]}"
done
sleep 4

if [[ "$RATE" != "1.0" && "$RATE" != "1" ]]; then
    echo "Note: rate $RATE changes the tracker timing (ticks are wall time). Use 1.0 for tuning."
fi
echo "Playing $BAG at rate $RATE"
ros2 bag play "$BAG" --clock --rate "$RATE" \
    --topics /tf /tf_static /"$NS"/landmarks /"$NS"/odom /"$NS"/pose

# The servers keep their final maps for inspection until Ctrl-C.
echo
echo "Playback finished. The servers keep running with their final maps;"
echo "inspect /tune_<label>/landmark_server/*, then press Ctrl-C to stop them."
trap 'exit 0' INT
while true; do sleep 1; done
