# Constants
uint32 CONTROL_MODE_TORQUE = 1
uint32 CONTROL_MODE_VELOCITY = 2
uint32 CONTROL_MODE_POSITION = 3
uint32 INPUT_MODE_PASSTHROUGH = 1
# Variables
uint32 control_mode
uint32 input_mode
float32 pos_setpoint
float32 vel_setpoint
float32 torq_setpoint