#!/bin/bash

PROJECT_NAME=selqie
PROJECT_DIR=/home/selqie/selqie_ws/src/

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Allow root to connect to the X server
if [ -n "$DISPLAY" ]; then
    xhost +local:root > /dev/null
fi

# Start the Docker container
docker run -it \
    --rm \
    --net host \
    --privileged \
    --gpus all \
    --name ${PROJECT_NAME} \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v "/${SCRIPT_DIR}/../:${PROJECT_DIR}" \
    ${PROJECT_NAME}:latest \
    /bin/bash