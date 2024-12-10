# SELQIE Robotic Software

|       Maintainer      |         E-mail           |
| --------------------- | ------------------------ |
| Jonathan Tyler Boylan | jtylerboylan@outlook.com |

<img src="./docs/figures/SoftwareStackNetwork.png"/>

## Packages

### [SELQIE ROS2](./selqie_ros2/)
- Contains the launch files for the different components of the SELQIE robot platform.
#### Terminal Mux
- Launches full robot suite with custom command terminal.
- SELQIE Hardware: `./src/tmux/selqie.sh`
- MuJoCo Simulation: `./src/tmux/selqie_mujoco.sh`

### [Bar100 Depth Sensor](./bar100_ros2/)
#### Requirements
-  Install Apt Packages:
```
sudo apt-get install python3-smbus
```
-  Install Bar100 Python Package:
```
git clone https://github.com/bluerobotics/KellerLD-python ~/.KellerLD
cd ~/.KellerLD && python3 setup.py install --user
```

#### Bar100 Node

- Reads the Bar100 depth sensor via I2C communication.
- Publishers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/depth` | `std_msgs/Float32` | Sensor depth reading |
| `/bar100/temperature` | `std_msgs/Float32` | Sensor temperature reading |

- Parameters:

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `i2c_bus` | `int` | `1` | I2C bus ID |
| `frequency` | `double` | `20.0` | Sensor publish frequency (in Hz) |
| `fluid_density` | `double` | `997.0474` | Density of the surrounding fluid (kg/m^3)|
| `gravity` | `double` | `9.80665` | Gravity (m/s^2) |
| `surface_pressure` | `double` | `1.0` | Pressure at the surface (bar) |

#### Depth2Pose Node

- Converts depth reading to a Pose with covariance. Used for EKF fusion in the `robot_localization` package.

- Publishers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output pose with covariance |

- Subscribers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/depth` | `std_msgs/Float32` | Input depth reading |

- Parameters:

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `frame_id` | `string` | `"odom"` | Frame of the depth reading |
| `z_variance` | `double` | `2.89` | Depth sensor variance |

### [Stereo USB Cameras](./stereo_usb_cam/)
#### Stereo USB Cam Node

### [Jetson GPIO](./jetson_ros2/)
#### GPIO Node

### [Terrain Mapping](./terrain_mapping/)
#### Terrain Mapping Node

### [Ground Plane Detection](./ground_plane_detection/)
#### Ground Plane Detection Node

### [Gait Planning](./gait_planning/)
#### Gait Planning Node

### [Local Planning](./local_planning/)
#### Local Planning Node

### [Stride Maker (2D)](./stride_maker/)
#### Walk 2D Node

### [Legged MPC (3D)](./legged_mpc/)
#### Body Trajectory Node
#### Foothold Planner Node
#### Legged MPC Node
#### Swing Leg Node

### [Leg Kinematics](./leg_kinematics/)
#### Five-bar 2D Node
#### Leg Trajectory Publisher Node

### [ODrive Motor Controllers](./odrive_ros2/)
#### ODrive CAN Node

### [CAN Communication](./can_bus/)
#### Requirements
- Install Apt Packages:
```
sudo apt install libsocketcan-dev can-utils
```
- Set up CAN boot service (Jetson AGX):
```
sudo cp ~/selqie_ws/src/tools/load_can.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable load_can.service
sudo systemctl start load_can.service
```
- Enable CAN interfaces (Jetson AGX):
```
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1
```
#### CAN Bus Node
- Receive and transmit frames on the CAN bus.
- Publishers

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/can/tx` | `robot_msgs/CanFrame` | CAN Frame to transmit on the bus |

- Subscribers 

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/can/rx` | `robot_msgs/CanFrame` | CAN Frame received on the bus |

- Parameters

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `interface` | `string` | `"can0"` | CAN Interface Name |

### [MuJoCo Simulation](./mujoco_ros2/)
#### MuJoCo Node

### [Robot Terminals](./robot_utils/)
#### SELQIE Robot Terminal
#### UnitreeA1 Robot Terminal

### [Joystick Controllers](./robot_joysticks/)
#### SELQIE Joystick

### [Experiment Nodes](./robot_experiments/)
#### Walk Stride Tracking Experiment
#### Walk Stride Sweep Experiment

### [Unitree A1 Robot](./unitree_a1_mujoco/)
#### Terminal Mux
- MuJoCo: `./src/tmux/unitree_a1.sh`

## Other Documentation

1. [Hardware Links for SELQIE](./docs/Hardware.md)
2. [Jetson AGX Flashing and Software Setup](./docs/JetsonAGX-Setup.md)

## Tools

1. [Configure an ODrive](./tools/configure_odrive.py)
2. [Flash the SELQIE Jetson](./tools/install.sh)
3. [Start the Jetson CAN Interfaces](./tools/loadcan_jetson.sh)