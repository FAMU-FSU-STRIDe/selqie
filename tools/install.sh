#!/bin/bash

# Exit on failure
set -e

# Check for develop flag
# True: Install on development computer (Ubuntu 22.04)
# False: Install on Jetson AGX Orin (Jetpack 6.1)
DEVEL_FLAG=false
case $arg in
  -d|--devel)
    DEVEL_FLAG=true
    shift
    ;;
esac

if [ "$DEVEL_FLAG" = true ]; then
    echo "Installing on development computer (Ubuntu 22.04)"
else
    echo "Installing on Jetson AGX Orin (Jetpack 6.1)"
fi

# OpenCV bug fix
if [ "$DEVEL_FLAG" = false ]; then
    sudo apt purge *libopencv*
    sudo apt remove opencv-licenses
    sudo apt install libopencv-dev=4.5.*
    sudo apt-mark hold libopencv-dev
fi

# Update the system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y  \
    curl wget gpg apt-transport-https gdb \
    python3-pip python3-smbus \
    libsocketcan-dev can-utils libeigen3-dev \
    libx11-dev xorg libglfw3 libglfw3-dev
if [ "$DEVEL_FLAG" = true ]; then
    # MuJoCo dependencies (Development Only)
    sudo apt install -y libx11-dev xorg libglfw3 libglfw3-dev
fi

# Install ROS2 Humble
ROS2_DISTRO="humble"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-${ROS2_DISTRO}-desktop

# Install ROS2 dependencies
pip3 install setuptools==58.1.0
sudo apt install -y \
    python3-colcon-common-extensions \
    ros-${ROS2_DISTRO}-camera-info-manager ros-${ROS2_DISTRO}-image-proc ros-${ROS2_DISTRO}-stereo-image-proc \
    ros-${ROS2_DISTRO}-robot-localization ros-${ROS2_DISTRO}-microstrain-inertial-driver ros-${ROS2_DISTRO}-grid-map

# Install MuJoCo (Development Only)
if [ "$DEVEL_FLAG" = true ]; then
    MUJOCO_PATH=${HOME}/.MuJoCo
    if [ ! -d ${MUJOCO_PATH} ]; then
        git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
        mkdir -p ${MUJOCO_PATH}/build && cd ${MUJOCO_PATH}/build && \
            cmake .. -DCMAKE_BUILD_TYPE=Release && \
            make && sudo make install
    fi
fi

# Install OSQP
OSQP_PATH=${HOME}/.OSQP
if [ ! -d ${OSQP_PATH} ]; then
    git clone https://github.com/osqp/osqp ${OSQP_PATH}
    mkdir -p ${OSQP_PATH}/build && cd ${OSQP_PATH}/build && \
        cmake -G "Unix Makefiles" .. -DCMAKE_BUILD_TYPE=Release && \
        make && sudo make install
fi

# Install SBMPO
SBMPO_PATH=${HOME}/.SBMPO
if [ ! -d ${SBMPO_PATH} ]; then
    git clone https://github.com/JTylerBoylan/sbmpo ${SBMPO_PATH}
    mkdir -p ${SBMPO_PATH}/build && cd ${SBMPO_PATH}/build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make && sudo make install
fi

# Install KellerLD (Bar100 Depth Sensor)
KELLERLD_PATH=${HOME}/.KellerLD
if [ ! -d ${KELLERLD_PATH} ]; then
    git clone https://github.com/bluerobotics/KellerLD-python ${KELLERLD_PATH}
    cd ${KELLERLD_PATH} && python3 setup.py install --user
fi

# Ignore MuJoCo package on Jetson AGX Orin
if [ "$DEVEL_FLAG" = false ]; then
    touch ${SELQIE_WS}/src/mujoco_ros2/COLCON_IGNORE
fi

# Build the project
SELQIE_WS=${HOME}/selqie_ws
source /opt/ros/${ROS2_DISTRO}/setup.bash && cd ${SELQIE_WS} && colcon build --symlink-install

# Source ROS2 in bashrc
ROS_SETUP="source /opt/ros/${ROS2_DISTRO}/setup.bash"
if ! grep -Fxq "$ROS_SETUP" ~/.bashrc; then
    echo "$ROS_SETUP" >> ~/.bashrc
fi

# Source project in bashrc
PROJECT_SETUP="source ${SELQIE_WS}/install/setup.bash"
if ! grep -Fxq "$PROJECT_SETUP" ~/.bashrc; then
    echo "$PROJECT_SETUP" >> ~/.bashrc
fi

# Jetson AGX Orin Setup
if [ "$DEVEL_FLAG" = false ]; then

    # Setup GPIO
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio ${USER}
    sudo cp ${SELQIE_WS}/src/tools/99-gpio.rules /etc/udev/rules.d/

    # Setup CAN Boot Service
    sudo cp ${SELQIE_WS}/src/tools/load_can.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable load_can.service
    sudo systemctl start load_can.service

    # Setup IMU Microstrain Rules
    sudo cp ${SELQIE_WS}/src/tools/100-microstrain.rules /etc/udev/rules.d/

    # Reload udev rules
    sudo udevadm control --reload-rules && sudo udevadm trigger

    # Setup GPIO Configuration
    sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1 pwm5
fi

echo "Setup complete!"