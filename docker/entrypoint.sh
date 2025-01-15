#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the workspace
cd ${SELQIE_WS} && colcon build --symlink-install

# Source the workspace
source "${SELQIE_WS}/install/setup.bash"

# Pass arguments
exec "$@"
