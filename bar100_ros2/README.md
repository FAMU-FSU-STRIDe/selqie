# Bar100 Depth Sensor ROS2 Package

## Requirements
### Install Apt Packages
```
sudo apt-get install python3-smbus
```
### Install Bar100 Python Package
```
git clone https://github.com/bluerobotics/KellerLD-python ~/.KellerLD
cd ~/.KellerLD && python3 setup.py install --user
```

## Bar100 Node

### Publishers

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/depth` | `std_msgs/Float32` | Sensor depth reading |
| `/bar100/temperature` | `std_msgs/Float32` | Sensor temperature reading |

### Parameters

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `i2c_bus` | `int` | `1` | I2C bus ID |
| `frequency` | `double` | `20.0` | Sensor publish frequency (in Hz) |
| `fluid_density` | `double` | `997.0474` | Density of the surrounding fluid (kg/m^3)|
| `gravity` | `double` | `9.80665` | Gravity (m/s^2) |
| `surface_pressure` | `double` | `1.0` | Pressure at the surface (bar) |


### Example Launch

```
Node(
    package='bar100_ros2',
    executable='bar100_node',
    name='bar100_node',
    output='screen',
    parameters=[{
        'i2c_bus': 7,
        'frequency': 20.0,
        'fluid_density': 997.0474,
        'gravity': 9.80665,
        'surface_pressure': 1.0,
    }],
)
```

## Depth2Pose Node

Converts depth reading from Bar100 Node to a Pose with covariance. Used for EKF fusion in the `robot_localization` package.

### Publishers

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output pose with covariance |

### Subscribers

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/depth` | `std_msgs/Float32` | Input depth reading |

### Parameters

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `frame_id` | `string` | `"odom"` | Frame of the depth reading |
| `z_variance` | `double` | `2.89` | Depth sensor variance |


### Example Launch

```
Node(
    package='bar100_ros2',
    executable='depth2pose_node',
    name='depth2pose_node',
    output='screen',
    parameters=[{
        'z_variance': 2.89,
    }],
)
```