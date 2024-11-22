# CAN BUS ROS2 Package

## Requirements
### Install Apt Packages
```
sudo apt install libsocketcan-dev can-utils
```
### Set up CAN boot service (Jetson AGX)
```
sudo cp ~/selqie_ws/src/tools/load_can.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable load_can.service
sudo systemctl start load_can.service
```
### Enable CAN interfaces (Jetson AGX)
```
sudo /opt/nvidia/jetson-io/config-by-function.py -o dt can0 can1
```

## CAN Bus Node

### Publishers

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/can/tx` | `robot_msgs/CanFrame` | CAN Frame to transmit on the bus |

### Subscribers 

| Topic | Message Type | Description |
| ----- | ------------ | ----------- |
| `/can/rx` | `robot_msgs/CanFrame` | CAN Frame received on the bus |

### Parameters

| Parameter | Type | Default Value | Description |
| --------- | ---- | ------------- | ----------- |
| `interface` | `string` | `"can0"` | CAN Interface Name |

### Example Launch

```
Node(
    package='can_bus',
    executable='can_bus_node',
    name='can0_node',
    parameters=[{
        'interface': 'can0'
    }],
    remappings=[
        ('can/tx', 'can0/tx'),
        ('can/rx', 'can0/rx')
    ]
)
```

### Troubleshooting

- Make sure the CAN interface is **UP** and **RUNNING** when running the command `$ ifconfig`. If not, see [`load_can.service`](../tools/load_can.service).

- Check that CAN frames are being received: `$ ros2 topic echo /can0/rx`.

- Make sure that the motors are on the correct CAN network.

- Check for loose wiring.

- Make sure there is a common ground connection between the CAN module, computer, and motors.

## CAN Bus Limits

The CAN bus has a max data rate of `1000 Kbps`, which limits the number of commands sent per second and the rate at which info is recieved from the ODrives.

- CAN bus frame size: `144 bits/frame`
- Frames per info: `6 frames/info`
- Info publish rate: `50 infos/second /motor`
- Motors per CAN bus: `4 motors`

`144 * 6 * 50 * 4` = **`172.8 Kbps`**

- Frames per command: `1 frames/command /motor`

`144 * 1 * 4` = **`0.576 Kbits/command`**

So, the maximum number of commands per second is
`(1000 - 172.8) / 0.576` \
= **`1436 commands/second`**