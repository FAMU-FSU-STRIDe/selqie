# Use ROS Humble as base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        git wget unzip \
        g++ cmake gdb \
        libsocketcan-dev can-utils \
        libeigen3-dev \
        libx11-dev xorg-dev libglfw3 libglfw3-dev \
        ros-humble-joy \
        ros-humble-rviz2

# Create a new user
ENV USERNAME=ubuntu
ENV HOME=/home/${USERNAME}
RUN useradd -m -u 1000 -s /bin/bash ${USERNAME} && mkdir -p ${HOME}

# Install MuJoCo
ENV MUJOCO_PATH=${HOME}/MuJoCo
RUN git clone https://github.com/google-deepmind/mujoco ${MUJOCO_PATH}
RUN mkdir ${MUJOCO_PATH}/build && \ 
    cd ${MUJOCO_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    cmake --build . --target install

# Install OSQP
ENV OSQP_PATH=${HOME}/osqp
RUN git clone https://github.com/osqp/osqp ${OSQP_PATH}
RUN mkdir ${OSQP_PATH}/build && \
    cd ${OSQP_PATH}/build && \
    cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --target install

# Give the user input group permissions
RUN groupadd -g 107 input && usermod -aG input ${USERNAME}

# Give the user sudo privileges
RUN echo "${USERNAME}:${USERNAME}" | chpasswd
RUN echo "${USERNAME} ALL=(ALL) ALL" >> /etc/sudoers

# Make user the owner of the home
RUN mkdir -p ${HOME}/ros2_ws && chown -R ${USERNAME}:${USERNAME} ${HOME}/ros2_ws

# Switch to the new non-root user
USER ${USERNAME}

RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to execute when creating a new container
CMD ["bash"]
