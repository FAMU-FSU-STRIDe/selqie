# Constants
uint8 AXIS_STATE_IDLE = 1
uint8 AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# Variables
uint8 axis_state
float32 gear_ratio
float32 vel_limit
float32 curr_limit
float32 pos_gain
float32 vel_gain
float32 vel_int_gain
bool clear_errors
