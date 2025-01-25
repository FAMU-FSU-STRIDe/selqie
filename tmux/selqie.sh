#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the ROS workspace
ROS2_WS=${SCRIPT_DIR}/../..

# Start a new tmux session
tmux new-session -d -s selqie

# Split the window into two panes (left and right)
tmux split-window -h

# Now select the right pane (which is pane 1) and split it horizontally to create top-right and bottom-right
tmux select-pane -t 0
tmux split-window -v

# Send the ros2 launch command to the left pane (pane 0)
tmux select-pane -t 0
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 launch selqie_ros2 selqie.launch.py" C-m

# The bottom-right pane (pane 2) will be left open for general commands, with the environment already sourced
tmux select-pane -t 1
tmux send-keys "source ${ROS2_WS}/install/setup.bash" C-m

# Send the ros2 run command to the top-right pane (pane 1)
tmux select-pane -t 2
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 run selqie_ros2 selqie_terminal.py" C-m

# Now source the tmux configuration file to enable bindings and mouse support
tmux source-file $SCRIPT_DIR/.tmux.conf

# Attach to the session
tmux attach-session -t selqie
