from robot_msgs.msg import *

def set_motor_states(cfg_publishers, axis_state):
    msg = ODriveConfig()
    msg.axis_state = axis_state
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def clear_motor_errors(cfg_publishers):
    msg = ODriveConfig()
    msg.clear_errors = True
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def set_motor_positions(cmd_publishers, pos):
    msg = MotorCommand()
    msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
    msg.pos_setpoint = pos
    for pub in cmd_publishers:
        pub.publish(msg)

def set_motor_gains(cfg_publishers, gains):
    msg = ODriveConfig()
    msg.pos_gain = gains[0]
    msg.vel_gain = gains[1]
    msg.vel_int_gain = gains[2]
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def print_motor_info(motor_info : ODriveInfo):
    for attr in ["axis_error", "axis_state", "bus_current", 
                 "bus_voltage", "fet_temperature", "motor_temperature",
                 "iq_measured", "iq_setpoint"]:
        print(f"  {attr}: {getattr(motor_info, attr)}")

def print_motor_estimate(motor_estimate : MotorEstimate):
    for attr in ["pos_estimate", "vel_estimate", "torq_estimate"]:
        print(f"  {attr}: {getattr(motor_estimate, attr)}")

def print_leg_info(leg_estimate : LegEstimate):
    for attr in ["pos_estimate", "vel_estimate", "force_estimate"]:
        vector = getattr(leg_estimate, attr)
        print(f"  {attr}: x={vector.x}, y={vector.y}, z={vector.z}")
        
def get_error_name(motor_info : ODriveInfo):
    for attr_name in dir(ODriveInfo):
        if attr_name.startswith("AXIS_ERROR_"):
            error_value = getattr(ODriveInfo, attr_name)
            if error_value == motor_info.axis_error:
                return attr_name
    return "UNKNOWN_ERROR"