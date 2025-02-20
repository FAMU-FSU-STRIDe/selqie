#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the ROS workspace
ROS2_WS=${SCRIPT_DIR}/../..

# Default values
ROSLAUNCH_ARGS="use_bag_ext:=false"
BASE_TOPIC_REMAPPINGS="/stereo/left/image_raw:=/stereo/left/image_raw/playback \
    /stereo/right/image_raw:=/stereo/right/image_raw/playback \
    /stereo/left/camera_info:=/stereo/left/camera_info/playback \
    /stereo/right/camera_info:=/stereo/right/camera_info/playback"
EXTENDED_TOPIC_REMAPPINGS=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -ext)
            ROSLAUNCH_ARGS="use_bag_ext:=true"
            EXTENDED_TOPIC_REMAPPINGS="/goal_pose/local:=/goal_pose/local/playback \
                /gait_planner/path:=/gait_planner/path/playback \
                /walk_planner/path:=/walk_planner/path/playback \
                /cmd_vel:=/cmd_vel/playback \
                /cmd_vel/raw:=/cmd_vel/raw/playback \
                /gait/vel_estimate:=/gait/vel_estimate/playback \
                /gait/transition:=/gait/transition/playback"
            shift
            ;;
        *)
            ROSBAG_PATH="$1" # Assume first non-option argument is the rosbag path
            shift
            ;;
    esac
done

# Ensure ROSBAG_PATH is set
if [ -z "$ROSBAG_PATH" ]; then
    echo "Error: No rosbag path provided."
    echo "Usage: $0 [-ext] <rosbag_path>"
    exit 1
fi

# Start a new tmux session
tmux new-session -d -s selqie

# Split the window into a 2x2 grid
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# Launch SELQIE in top-left with correct launch arguments
tmux select-pane -t 0
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 launch selqie_ros2 selqie_playback.launch.py ${ROSLAUNCH_ARGS}" C-m

# Sourced environment in bottom-left
tmux select-pane -t 1
tmux send-keys "source ${ROS2_WS}/install/setup.bash" C-m

# SELQIE Terminal in top-right
tmux select-pane -t 2
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 run selqie_ros2 selqie_terminal.py" C-m

# Jetson Stats or ROSBAG Playback in bottom-right
tmux select-pane -t 3
tmux send-keys "source ~/selqie_ws/install/setup.bash; ros2 bag play $ROSBAG_PATH --clock --remap $BASE_TOPIC_REMAPPINGS $EXTENDED_TOPIC_REMAPPINGS" C-m

# Source tmux config file to enable bindings and mouse support
tmux source-file $SCRIPT_DIR/.tmux.conf

# Attach to the session
tmux attach-session -t selqie
