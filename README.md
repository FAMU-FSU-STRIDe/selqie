# SELQIE Robotic Software

|       Maintainer      |         E-mail           |
| --------------------- | ------------------------ |
| Jonathan Tyler Boylan | jtylerboylan@outlook.com |

<img src="./docs/figures/SoftwareStackNetwork.png"/>

## Documentation

1. [Hardware Links for SELQIE](./docs/Hardware.md)
2. [Jetson AGX Flashing and Software Setup](./docs/JetsonAGX-Setup.md)

## Tools

1. [Configure an ODrive](./tools/configure_odrive.py)
2. [Flash the SELQIE Jetson](./tools/install.sh)
3. [Start the Jetson CAN Interfaces](./tools/loadcan_jetson.sh)

## Custom Packages

### Sensing and Perception
#### [Bar100 Depth Sensor](./bar100_ros2/)
#### [Stereo USB Cameras](./stereo_usb_cam/)

### Mapping
#### [Terrain Mapping](./terrain_mapping/)
#### [Ground Plane Detection](./ground_plane_detection/)

### Planning
#### [SBMPO Gait Planning](./gait_planning/)
#### [SBMPO Local Planning](./local_planning/)

### Leg Control
#### [2DOF Leg Stride Maker](./stride_maker/)
#### [3DOF Leg Body MPC](./legged_mpc/)
#### [Leg Kinematics](./leg_kinematics/)

### Motor Actuation
#### [CAN Communication](./can_bus/)
#### [ODrive Motor Controllers](./odrive_ros2/)

### Simulation
#### [MuJoCo Simulation](./mujoco_ros2/)

### General
#### [SELQIE Robot](./selqie_ros2/)
#### [Unitree A1 Robot](./unitree_a1_mujoco/)
#### [Joystick Controllers](./robot_joysticks/)
#### [Experiment Nodes](./robot_experiments/)
#### [Robot Terminals](./robot_utils/)
#### [Jetson GPIO](./jetson_ros2/)

### Robot Terminals
Run these commands from `~/selqie_ws` to start a tmux terminal for a specific robot:
- SELQIE (Hardware): `./src/tmux/selqie.sh`
- SELQIE (Simulation): `./src/tmux/selqie_mujoco.sh`
- Unitree A1 (Simulation): `./src/tmux/unitree_a1.sh`