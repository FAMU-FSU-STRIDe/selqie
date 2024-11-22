# Jetson AGX ROS2 Package

## Requirements
### Set up User Permissions
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio ${USER}
sudo cp ~/selqie_ws/src/tools/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Enable PWM interfaces
- [Configuring Expansion Header](https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html)
- [Configuring using CLI](https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html#command-line-interface)
```
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt pwm5
```

## GPIO Node

### Publishers
| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/gpio/out` | `std_msgs/UInt8` | Signal to send on GPIO pin |

### Subscribers 
| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/gpio/in` | `std_msgs/UInt8` | Signal read by GPIO pin |

### Parameters
| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `gpio_pin` | `int` | `18` | Pin number ([Jetson Pinout](https://jetsonhacks.com/nvidia-jetson-agx-orin-gpio-header-pinout/)) |
| `is_output` | `bool` | `True` | IO mode |
| `is_pwm` | `bool` | `False` | PWM mode (Output only) |
| `frequency` | `double` | `50.0` | Read frequency or PWM frequency |
| `initial_value` | `int` | `0` | Default output value |

### Example Launch

```
Node(
    package='jetson_ros2',
    executable='gpio_node',
    name='camera_light_node',
    output='screen',
    parameters=[{
        'gpio_pin': 18,
        'is_pwm': True,
    }],
    remappings=[
        ('gpio/out', 'light/pwm'),
    ]
)
```