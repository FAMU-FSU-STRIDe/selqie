from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo

def set_odrive_states(cfg_publishers, axis_state):
    msg = MotorConfig()
    msg.axis_state = axis_state
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def clear_odrive_errors(cfg_publishers):
    msg = MotorConfig()
    msg.clear_errors = True
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def set_odrive_positions(cmd_publishers, pos):
    msg = MotorCommand()
    msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
    msg.pos_setpoint = pos
    for pub in cmd_publishers:
        pub.publish(msg)

def set_odrive_gains(cfg_publishers, gains):
    msg = MotorConfig()
    msg.pos_gain = gains[0]
    msg.vel_gain = gains[1]
    msg.vel_int_gain = gains[2]
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)