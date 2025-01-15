FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update
RUN apt-get install -y \
    git curl wget gpg apt-transport-https gdb tmux \
    python3-pip python3-smbus \
    libsocketcan-dev can-utils libeigen3-dev

# Install ROS dependencies
RUN pip3 install setuptools==58.1.0
RUN apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-camera-info-manager ros-humble-image-proc ros-humble-stereo-image-proc \
    ros-humble-robot-localization ros-humble-microstrain-inertial-driver ros-humble-grid-map

# Create user
RUN useradd -ms /bin/bash selqie
ENV HOME=/home/selqie

# Install MuJoCo dependencies
RUN apt install -y \
    libx11-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev

# Install MuJoCo
ENV MUJOCO_PATH=${HOME}/.MuJoCo
RUN git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
RUN mkdir -p ${MUJOCO_PATH}/build && cd ${MUJOCO_PATH}/build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make && make install

# Install OSQP
ENV OSQP_PATH=${HOME}/.OSQP
RUN git clone https://github.com/osqp/osqp ${OSQP_PATH}
RUN mkdir -p ${OSQP_PATH}/build && cd ${OSQP_PATH}/build && \
        cmake -G "Unix Makefiles" .. -DCMAKE_BUILD_TYPE=Release && \
        make && make install

# Install SBMPO
ENV SBMPO_PATH=${HOME}/.SBMPO
RUN git clone https://github.com/JTylerBoylan/sbmpo ${SBMPO_PATH}
RUN mkdir -p ${SBMPO_PATH}/build && cd ${SBMPO_PATH}/build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make && make install

# Install KellerLD (Depth sensor driver)
ENV KELLERLD_PATH=${HOME}/.KellerLD
RUN git clone https://github.com/bluerobotics/KellerLD-python ${KELLERLD_PATH}
RUN cd ${KELLERLD_PATH} && python3 setup.py install --user

# Setup entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

# Set user and workspace
USER selqie
ENV SELQIE_WS=${HOME}/selqie_ws
RUN mkdir ${SELQIE_WS}
WORKDIR ${SELQIE_WS}

# Enter terminal
CMD [ "/bin/bash" ]