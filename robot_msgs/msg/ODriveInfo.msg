# Constants
uint32 AXIS_ERROR_MISSING_INPUT = 64
uint32 AXIS_ERROR_DC_BUS_OVER_VOLTAGE = 256
uint32 AXIS_ERROR_DC_BUS_UNDER_VOLTAGE = 512
uint32 AXIS_ERROR_BUS_OVER_CURRENT = 1024
uint32 AXIS_ERROR_MOTOR_OVER_TEMP = 8192
uint32 AXIS_ERROR_ESTOP_REQUESTED = 33554432

# Variables
uint32 axis_error
uint8 axis_state
float32 iq_setpoint
float32 iq_measured
float32 fet_temperature
float32 motor_temperature
float32 bus_voltage
float32 bus_current