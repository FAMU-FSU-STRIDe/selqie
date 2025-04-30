# Constants
uint32 CONTROL_MODE_FORCE = 1
uint32 CONTROL_MODE_VELOCITY = 2
uint32 CONTROL_MODE_POSITION = 3

# Variables
uint32 control_mode
geometry_msgs/Vector3 pos_setpoint
geometry_msgs/Vector3 vel_setpoint
geometry_msgs/Vector3 force_setpoint