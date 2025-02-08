#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/selqie_ws/install/setup.bash
ros2 bag play $1 --clock --remap \
    /stereo/left/image_raw:=/stereo/left/image_raw/playback \
    /stereo/right/image_raw:=/stereo/right/image_raw/playback \
    /stereo/left/camera_info:=/stereo/left/camera_info/playback \
    /stereo/right/camera_info:=/stereo/right/camera_info/playback \
    "${@:2}"

