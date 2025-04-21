#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the ROS workspace
ROS2_WS=${SCRIPT_DIR}/../..

# Start a new tmux session
tmux new-session -d -s selqie

# Split the window into a 2x2 grid
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# Launch SELQIE (Simulation) in top-left
tmux select-pane -t 0
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 launch selqie_bringup selqie_sim.launch.py" C-m

# Sourced environment in bottom-left
tmux select-pane -t 1
tmux send-keys "source ${ROS2_WS}/install/setup.bash" C-m

# SELQIE Terminal in top-right
tmux select-pane -t 2
tmux send-keys "source ${ROS2_WS}/install/setup.bash; ros2 run selqie_ui selqie_terminal" C-m

# Sourced environment in bottom-right
tmux select-pane -t 3
tmux send-keys "source ${ROS2_WS}/install/setup.bash" C-m

# Source tmux config file to enable bindings and mouse support
tmux source-file $SCRIPT_DIR/.tmux.conf

# Attach to the session
tmux attach-session -t selqie
