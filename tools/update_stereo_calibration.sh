#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Extract the calibration data
mkdir -p /tmp/calibrationdata
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calibrationdata

# Move left and right calibration files to config directory
mv /tmp/calibrationdata/left.yaml ${SCRIPT_DIR}/../vision/vision_bringup/config/calibration_left.yaml
mv /tmp/calibrationdata/right.yaml ${SCRIPT_DIR}/../vision/vision_bringup/config/calibration_right.yaml