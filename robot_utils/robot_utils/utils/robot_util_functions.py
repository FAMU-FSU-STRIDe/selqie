import os
import time
from dataclasses import dataclass, field
from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo

TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')

@dataclass
class Trajectory:
    size: int = 0
    frequency: float = 0.0
    num_legs: int = 4
    delay_time: list = field(default_factory=list)
    leg_ids: list = field(default_factory=list)
    control_modes: list = field(default_factory=list)
    input_modes: list = field(default_factory=list)
    positions: list = field(default_factory=list)
    velocities: list = field(default_factory=list)
    forces: list = field(default_factory=list)

def set_motor_states(cfg_publishers, axis_state):
    msg = MotorConfig()
    msg.axis_state = axis_state
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
def clear_motor_errors(cfg_publishers):
    msg = MotorConfig()
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
    msg = MotorConfig()
    msg.pos_gain = gains[0]
    msg.vel_gain = gains[1]
    msg.vel_int_gain = gains[2]
    for cfg_pub in cfg_publishers:
        cfg_pub.publish(msg)
        
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

def get_trajectory_from_file(rel_file : str, frequency : float, num_legs : int = 4) -> Trajectory:
    file = os.path.join(TRAJECTORIES_FOLDER, rel_file)
    if not os.path.exists(file):
        print('File ' + file + ' does not exist')
        return
    
    trajectory = Trajectory()
    trajectory.frequency = frequency
    trajectory.num_legs = num_legs
    with open(file) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 13:
                print('Invalid file line: ' + line)
                return
            trajectory.size += 1
            trajectory.delay_time.append(float(parts[0]) / 1000.0 / frequency)
            trajectory.leg_ids.append(int(parts[1]))
            trajectory.control_modes.append(int(parts[2]))
            trajectory.input_modes.append(int(parts[3]))
            trajectory.positions.append([float(parts[4]), float(parts[5]), float(parts[6])])
            trajectory.velocities.append([float(parts[7]), float(parts[8]), float(parts[9])])
            trajectory.forces.append([float(parts[10]), float(parts[11]), float(parts[12])])
    if (max(trajectory.leg_ids) > num_legs) or (min(trajectory.leg_ids) < 0):
        print(f'Expected leg ids between 0 and {num_legs - 1}')
        return
    trajectory.delay_time, trajectory.leg_ids, trajectory.control_modes, trajectory.input_modes, \
        trajectory.positions, trajectory.velocities, trajectory.forces = zip(
            *sorted(zip(trajectory.delay_time, trajectory.leg_ids, 
                        trajectory.control_modes, trajectory.input_modes, 
                        trajectory.positions, trajectory.velocities, trajectory.forces)))
    return trajectory

def run_leg_trajectory_file(cmd_publishers : list, file : str, num_loops : int, frequency : float):
    trajectory = get_trajectory_from_file(file, frequency)
    print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
    for i in range(num_loops):
        print(f"  Loop {i+1}/{num_loops}")
        cstart = time.time()
        for i in range(trajectory.size):
            cnow = time.time()
            while cnow - cstart < trajectory.delay_time[i]:
                cnow = time.time()
            set_leg_states([cmd_publishers[trajectory.leg_ids[i]]], trajectory.control_modes[i], 
                           trajectory.positions[i], trajectory.velocities[i], trajectory.forces[i])
    print("Finished trajectory")

