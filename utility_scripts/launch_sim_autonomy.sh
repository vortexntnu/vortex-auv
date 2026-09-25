#!/bin/bash
# Launch the simulator with the landmark/navigation chain in a tmux session:
# Stonefish, the DP controller with the reference filter, landmark_server,
# waypoint_manager, the dummy perception (robosub_dummy_publisher) and the
# Foxglove bridge. When the vehicle is up it turns on autonomous mode and sets
# the course frame, so goals and landmark_targets scenarios can be sent.
# Usage: ./launch_sim_autonomy.sh [OPTIONS]   (see --help)

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --scenario <name>     Stonefish scenario (default: robosub)
  --seed <n>            Seed for the course roles, used by both the sim and
                        the dummy perception so they agree (default: 7)
  --domain-id <id>      ROS_DOMAIN_ID to use (default: 0)
  --fov                 Dummy perception only publishes what the cameras can
                        see from the vehicle pose (default: everything)
  --unstable            Dummy perception is noisy and drops out (profile
                        unstable: misses, occlusions, outliers, clutter)
  --moving <sec>        The jars and containers on the table move, on average
                        every <sec> seconds each (default: 0, they stay put)
  --tasks <list>        Comma-separated course elements for the dummy
                        perception, e.g. gate,slalom (default: all)
  --full-res            Render the sim at 1920x1080 high quality (default:
                        960x540 low; the full course can run out of memory)
  --mem-limit <GB>      Memory limit for the simulator, 0 for none (default: 5)
  --keyboard-joy <bool> Drive with the keyboard (keyboard_joy) next to the
                        joystick interface, which always runs (default: true)
  --no-autonomy         Do not turn on autonomous mode or set the course frame
  --headless            No rendering and no course geometry (scenario
                        nautilus_no_gpu): light, for testing the landmark
                        chain with the dummy perception. No camera images
  --drift <deg/m>       Odometry that drifts <deg/m> of yaw per metre, and
                        camera noise on the detections (drift_injector.py).
                        landmark_server runs on the drifting odometry, a
                        second one without the graph (/nautilus_raw) on the
                        same data for comparison. Implies --fov. The
                        controller still steers on the true odometry
  --detach              Start the session without attaching to it
  -h, --help            Show this help message

Windows: sim (simulator, controller, landmark_server, waypoint_manager),
perception (dummy perception, detection markers and frames), check
(graph_eval: the map against the true course; with --drift also the drift
injector and the server without graph), tools (Foxglove bridge, commands).
Foxglove layout: mission/landmark_server/foxglove/landmark_graph.json.
Detach with Ctrl-b d; stop everything with
  tmux kill-session -t sim_autonomy
EOF
}

SCENARIO="robosub"
SEED="7"
DOMAIN_ID="0"
FOV="false"
UNSTABLE="false"
MOVING="0.0"
TASKS=""
FULL_RES="false"
MEM_LIMIT="5"
AUTONOMY="true"
KEYBOARD_JOY="true"
HEADLESS="false"
DRIFT=""
DETACH="false"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario)    SCENARIO="$2";  shift 2 ;;
        --seed)        SEED="$2";      shift 2 ;;
        --domain-id)   DOMAIN_ID="$2"; shift 2 ;;
        --fov)         FOV="true";     shift ;;
        --unstable)    UNSTABLE="true"; shift ;;
        --moving)      MOVING="$2";    shift 2 ;;
        --tasks)       TASKS="$2";     shift 2 ;;
        --full-res)    FULL_RES="true"; shift ;;
        --mem-limit)   MEM_LIMIT="$2"; shift 2 ;;
        --no-autonomy) AUTONOMY="false"; shift ;;
        --keyboard-joy) KEYBOARD_JOY="$2"; shift 2 ;;
        --headless)    HEADLESS="true"; shift ;;
        --drift)       DRIFT="$2"; FOV="true"; shift 2 ;;
        --detach)      DETACH="true"; shift ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

# ROS parameters are typed: "10" would be an integer where a double is expected.
if [[ "$MOVING" =~ ^[0-9]+$ ]]; then
    MOVING="$MOVING.0"
fi

# The workspace is three levels up: <ws>/src/vortex-auv/utility_scripts.
WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
    echo "No install/setup.bash in $WS; build the workspace first."
    exit 1
fi

SESSION="sim_autonomy"
S="cd $WS && source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"
DUMMY_CONFIG="install/robosub_dummy_publisher/share/robosub_dummy_publisher/config"

if [[ -n "$DRIFT" ]] && ! [[ "$DRIFT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--drift takes degrees per metre, e.g. 0.5"
    exit 1
fi
if [[ "$DRIFT" =~ ^[0-9]+$ ]]; then
    DRIFT="$DRIFT.0"
fi

# Simulator: low resolution and a memory limit unless asked otherwise.
if [[ "$HEADLESS" == "true" ]]; then
    SIM_ARGS="keyboard_joy:=false rendering:=false scenario:=nautilus_no_gpu"
else
    SIM_ARGS="keyboard_joy:=$KEYBOARD_JOY scenario:=$SCENARIO robosub_icon_seed:=$SEED"
    if [[ "$FULL_RES" != "true" ]]; then
        SIM_ARGS="$SIM_ARGS window_res_x:=960 window_res_y:=540 rendering_quality:=low"
    fi
fi
SIM_CMD="ros2 launch stonefish_sim vortex_sim_launch.py $SIM_ARGS"
if [[ "$MEM_LIMIT" != "0" ]] && command -v systemd-run &>/dev/null; then
    SIM_CMD="systemd-run --user --scope -p MemoryMax=${MEM_LIMIT}G -p MemorySwapMax=0 $SIM_CMD"
fi

# Dummy perception: the unstable profile goes on top of the defaults, the
# command line parameters on top of both.
DUMMY_CMD="ros2 run robosub_dummy_publisher robosub_dummy_publisher_node --ros-args -r __ns:=/nautilus"
DUMMY_CMD="$DUMMY_CMD --params-file $DUMMY_CONFIG/robosub_dummy_publisher_params.yaml"
if [[ "$UNSTABLE" == "true" ]]; then
    DUMMY_CMD="$DUMMY_CMD --params-file $DUMMY_CONFIG/robosub_dummy_publisher_unstable.yaml"
fi
DUMMY_CMD="$DUMMY_CMD -p seed:=$SEED -p use_field_of_view:=$FOV -p movable_move_interval_sec:=$MOVING"
if [[ -n "$TASKS" ]]; then
    DUMMY_CMD="$DUMMY_CMD -p tasks:=[$TASKS]"
fi

# landmark_server. With --drift: on the drifting odometry (the detections
# come from the drift injector on the usual topic), plus a second server
# without the graph for comparison.
LS_CONFIG="install/landmark_server/share/landmark_server/config"
LS_PARAMS="--params-file $LS_CONFIG/landmark_server_config.yaml --params-file $LS_CONFIG/sim.yaml --params-file install/auv_setup/share/auv_setup/config/robots/nautilus.yaml"
LS_CMD="ros2 launch landmark_server landmark_server.launch.py env:=sim"
EVAL_CMD="ros2 run landmark_server graph_eval.py --ros-args -p truth_seed:=$SEED -p maps:=[/nautilus/landmark_server/object_map] -p labels:=[graph] -p csv:=/tmp/graph_eval.csv"
if [[ -n "$DRIFT" ]]; then
    DUMMY_CMD="$DUMMY_CMD -p topic:=landmarks_true"
    INJECT_CMD="ros2 run landmark_server drift_injector.py --ros-args -p drift_yaw_deg_per_m:=$DRIFT -p noise:=true -p landmarks_out:=/nautilus/landmarks"
    LS_CMD="ros2 run landmark_server landmark_server_node --ros-args -r __ns:=/nautilus $LS_PARAMS -p topics.odom:=/nautilus/odom_drift"
    RAW_CMD="ros2 run landmark_server landmark_server_node --ros-args -r __ns:=/nautilus_raw $LS_PARAMS -p topics.odom:=/nautilus/odom_drift -p topics.landmarks:=/nautilus/landmarks -p graph.enable:=false -p course_frame.publish_tf:=false"
    EVAL_CMD="ros2 run landmark_server graph_eval.py --ros-args -p truth_seed:=$SEED -p maps:=[/nautilus/landmark_server/object_map,/nautilus_raw/landmark_server/object_map] -p labels:=[graph,raw] -p csv:=/tmp/graph_eval.csv"
fi

# Frames and detection markers for Foxglove (see foxglove_helpers.launch.py).
FRAMES_CMD="ros2 launch robosub_dummy_publisher foxglove_helpers.launch.py"

# Once the vehicle publishes odometry: autonomous mode and the course frame.
# The wait does not use the ros2 daemon, which can be stuck after a restart.
AUTONOMY_CMD="echo 'Waiting for /nautilus/odom ...'
until timeout 5 ros2 topic echo --no-daemon /nautilus/odom nav_msgs/msg/Odometry --once --no-arr >/dev/null 2>&1; do sleep 2; done
ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'
ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'
ros2 service call /nautilus/landmark_server/set_course_frame vortex_msgs/srv/SetCourseFrame '{start_pose: {orientation: {w: 1.0}}, heading_offset_rad: 0.0}'
clear
echo 'Autonomous mode on, course frame set. Try:'
echo '  ros2 run landmark_targets landmark_targets_scenario_node --ros-args -r __ns:=/nautilus -p scenario:=gate'
echo '  ros2 service call /nautilus/landmark_server/clear std_srvs/srv/Empty'
echo '  ros2 run landmark_server drift_route.py    # a loop past the course (route:=long for two laps)'"
HELP_CMD="clear && echo 'Turn on autonomous mode yourself:' && echo \"  ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'\" && echo \"  ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'\""

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# Launch Foxglove Studio only if not already running
if ! pgrep -f foxglove-studio &>/dev/null && command -v foxglove-studio &>/dev/null; then
    foxglove-studio &>/dev/null &
fi

# =============================================
# Window 1: sim (4 panes)
# =============================================
tmux new-session -d -s "$SESSION" -n "sim"

PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')
tmux send-keys -t "$PANE_SIM" "clear && $S && $SIM_CMD" Enter

PANE_CTRL=$(tmux split-window -h -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_CTRL" "clear && $S && ros2 launch auv_setup dp_quat.launch.py" Enter

PANE_MAP=$(tmux split-window -v -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_MAP" "clear && $S && $LS_CMD" Enter

PANE_WM=$(tmux split-window -v -t "$PANE_CTRL" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_WM" "clear && $S && ros2 launch waypoint_manager waypoint_manager.launch.py" Enter

tmux select-layout -t "$SESSION:sim" tiled

# =============================================
# Window 2: perception (2 panes)
# =============================================
tmux new-window -t "$SESSION" -n "perception"

PANE_DUMMY=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_DUMMY" "clear && $S && $DUMMY_CMD" Enter

PANE_FRAMES=$(tmux split-window -v -t "$PANE_DUMMY" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_FRAMES" "clear && $S && $FRAMES_CMD" Enter

# =============================================
# Window 3: check (the map against the true course)
# =============================================
tmux new-window -t "$SESSION" -n "check"

PANE_EVAL=$(tmux list-panes -t "$SESSION:check" -F '#{pane_id}')
tmux send-keys -t "$PANE_EVAL" "clear && $S && sleep 10 && $EVAL_CMD" Enter
if [[ -n "$DRIFT" ]]; then
    PANE_INJECT=$(tmux split-window -v -t "$PANE_EVAL" -P -F '#{pane_id}')
    tmux send-keys -t "$PANE_INJECT" "clear && $S && $INJECT_CMD" Enter
    PANE_RAW=$(tmux split-window -h -t "$PANE_INJECT" -P -F '#{pane_id}')
    tmux send-keys -t "$PANE_RAW" "clear && $S && $RAW_CMD" Enter
fi

# =============================================
# Window 4: tools (2 panes)
# =============================================
tmux new-window -t "$SESSION" -n "tools"

PANE_FOX=$(tmux list-panes -t "$SESSION:tools" -F '#{pane_id}')
tmux send-keys -t "$PANE_FOX" "clear && $S && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

PANE_CMD=$(tmux split-window -v -t "$PANE_FOX" -P -F '#{pane_id}')
if [[ "$AUTONOMY" == "true" ]]; then
    tmux send-keys -t "$PANE_CMD" "clear && $S && $AUTONOMY_CMD" Enter
else
    tmux send-keys -t "$PANE_CMD" "$S && $HELP_CMD" Enter
fi

# =============================================
# Focus the command pane and attach
# =============================================
tmux select-window -t "$SESSION:tools"
tmux select-pane -t "$PANE_CMD"
if [[ "$DETACH" == "true" ]]; then
    echo "Session $SESSION started; attach with: tmux attach -t $SESSION"
elif [[ -n "$TMUX" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
