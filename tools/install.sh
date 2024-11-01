#!/bin/bash

# Update the system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y  \
    curl wget gpg apt-transport-https gdb python3-pip \
    libsocketcan-dev can-utils libeigen3-dev libx11-dev xorg libglfw3 libglfw3-dev
pip3 install setuptools==58.1.0

# Install ROS2 Humble
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop

# Install ROS2 dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-grid-map ros-humble-camera-info-manager ros-humble-image-proc ros-humble-stereo-image-proc

# Install MuJoCo
export MUJOCO_PATH=${HOME}/.MuJoCo
if [ ! -d ${MUJOCO_PATH} ]; then
    git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
    mkdir -p ${MUJOCO_PATH}/build && cd ${MUJOCO_PATH}/build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make && sudo make install
fi

# Install OSQP
export OSQP_PATH=${HOME}/.OSQP
if [ ! -d ${OSQP_PATH} ]; then
    git clone https://github.com/osqp/osqp ${OSQP_PATH}
    mkdir -p ${OSQP_PATH}/build && cd ${OSQP_PATH}/build && \
        cmake -G "Unix Makefiles" .. -DCMAKE_BUILD_TYPE=Release && \
        make && sudo make install
fi

# Install SBMPO
export SBMPO_PATH=${HOME}/.SBMPO
if [ ! -d ${SBMPO_PATH} ]; then
    git clone https://github.com/JTylerBoylan/sbmpo ${SBMPO_PATH}
    mkdir -p ${SBMPO_PATH}/build && cd ${SBMPO_PATH}/build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make && sudo make install
fi

# Build the project
export SELQIE_WS=${HOME}/selqie_ws
source /opt/ros/humble/setup.bash && cd ${SELQIE_WS} && colcon build --symlink-install

# Source ROS2 in bashrc
ROS_SETUP="source /opt/ros/humble/setup.bash"
if ! grep -Fxq "$ROS_SETUP" ~/.bashrc; then
    echo "$ROS_SETUP" >> ~/.bashrc
fi

# Source project in bashrc
PROJECT_SETUP="source ${SELQIE_WS}/install/setup.bash"
if ! grep -Fxq "$PROJECT_SETUP" ~/.bashrc; then
    echo "$PROJECT_SETUP" >> ~/.bashrc
fi

# Setup CAN Boot Service
sudo cp ${SELQIE_WS}/src/tools/load_can.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable load_can.service
sudo systemctl start load_can.service

# Setup IMU Microstrain Rules
sudo cp ${SELQIE_WS}/src/tools/100-microstrain.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules

# Setup GPIO Configuration
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5

# Install VSCode
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg
sudo apt update
sudo apt install code