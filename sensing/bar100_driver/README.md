# Bar100 Driver Notes

### Links
- [Shop / Documents](https://bluerobotics.com/store/sensors-cameras/sensors/bar100-sensor-r2-rp/)
- [Driver Source Code](https://github.com/bluerobotics/KellerLD-python)

### Requirements
-  Install Apt Packages:
```
sudo apt install python3-smbus
```
-  Install Bar100 Python Package:
```
git clone https://github.com/bluerobotics/KellerLD-python ~/.KellerLD
cd ~/.KellerLD && python3 setup.py install --user
```

### Bar100 Node

- Reads the Bar100 pressure sensor via I2C communication.
- Publishers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/pressure` | `std_msgs/Float32` | Sensor pressure reading |
| `/bar100/temperature` | `std_msgs/Float32` | Sensor temperature reading |

- Parameters:

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `i2c_bus` | `int` | `1` | I2C bus ID |
| `frequency` | `double` | `20.0` | Sensor publish frequency (in Hz) |

### Depth2Pose Node

- Converts depth reading to a Pose with covariance. Used for EKF fusion in the `robot_localization` package.

- Publishers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Output pose with covariance |

- Subscribers:

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/bar100/pressure` | `std_msgs/Float32` | Input pressure reading |

- Parameters:

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `frame_id` | `string` | `"odom"` | Frame of the depth reading |
| `fluid_density` | `double` | `997.0474` | Density of the surrounding fluid (kg/m^3)|
| `gravity` | `double` | `9.80665` | Gravity (m/s^2) |
| `z_variance` | `double` | `0.0004` | Depth sensor variance |