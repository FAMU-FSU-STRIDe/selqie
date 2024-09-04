import time
from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo

def set_leg_states(cmd_publishers, mode, pos, vel = [0.0, 0.0, 0.0], force = [0.0, 0.0, 0.0]):
    msg = LegCommand()
    msg.control_mode = mode
    msg.pos_setpoint.x = pos[0]
    msg.pos_setpoint.y = pos[1]
    msg.pos_setpoint.z = pos[2]
    msg.vel_setpoint.x = vel[0]
    msg.vel_setpoint.y = vel[1]
    msg.vel_setpoint.z = vel[2]
    msg.force_setpoint.x = force[0]
    msg.force_setpoint.y = force[1]
    msg.force_setpoint.z = force[2]
    for cmd_pub in cmd_publishers:
        cmd_pub.publish(msg)

def get_trajectory_from_file(file : str, frequency : float, num_legs : int = 4):
    delay_time = []
    leg_ids = []
    control_modes = []
    input_modes = [] # not used
    positions = []
    velocities = []
    forces = []
    with open(file) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 13:
                print('Invalid file line: ' + line)
                return
            delay_time.append(float(parts[0]) / 1000.0 / frequency)
            leg_ids.append(int(parts[1]))
            control_modes.append(int(parts[2]))
            input_modes.append(int(parts[3]))
            positions.append([float(parts[4]), float(parts[5]), float(parts[6])])
            velocities.append([float(parts[7]), float(parts[8]), float(parts[9])])
            forces.append([float(parts[10]), float(parts[11]), float(parts[12])])
    if (max(leg_ids) > num_legs) or (min(leg_ids) < 0):
        print(f'Expected leg ids between 0 and {num_legs - 1}')
        return
    return zip(*sorted(zip(delay_time, leg_ids, control_modes, input_modes, positions, velocities, forces)))

def run_leg_trajectory_file(cmd_publishers : list, file : str, num_loops : int, frequency : float):
    delay_time, leg_ids, control_modes, input_modes, positions, velocities, forces = get_trajectory_from_file(file, frequency)
    print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
    for i in range(num_loops):
        print(f"  Loop {i+1}/{num_loops}")
        cstart = time.time()
        for i in range(len(delay_time)):
            cnow = time.time()
            while cnow - cstart < delay_time[i]:
                cnow = time.time()
            set_leg_states([cmd_publishers[leg_ids[i]]], control_modes[i], positions[i], velocities[i], forces[i])
    print("Finished trajectory")
            
def print_motor_info(motor_info : MotorInfo, motor_estimate : MotorEstimate):
    for attr in ["axis_error", "axis_state", "bus_current", 
                 "bus_voltage", "fet_temperature", "motor_temperature",
                 "iq_measured", "iq_setpoint"]:
        print(f"  {attr}: {getattr(motor_info, attr)}")
    for attr in ["pos_estimate", "vel_estimate", "torq_estimate"]:
        print(f"  {attr}: {getattr(motor_estimate, attr)}")

def print_leg_info(leg_estimate : LegEstimate):
    for attr in ["pos_estimate", "vel_estimate", "force_estimate"]:
        vector = getattr(leg_estimate, attr)
        print(f"  {attr}: x={vector.x}, y={vector.y}, z={vector.z}")
        
def get_error_name(motor_info : MotorInfo):
    for attr_name in dir(MotorInfo):
        if attr_name.startswith("AXIS_ERROR_"):
            error_value = getattr(MotorInfo, attr_name)
            if error_value == motor_info.axis_error:
                return attr_name
    return "UNKNOWN_ERROR"

