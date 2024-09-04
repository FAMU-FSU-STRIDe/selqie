import os
import time
from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo

TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')

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

def run_leg_trajectory_file(file : str, num_loops : int, frequency : float, publishers : list):

    traj_file = os.path.join(TRAJECTORIES_FOLDER, file)
    if not os.path.exists(traj_file):
        print('File ' + traj_file + ' does not exist')
        return
    
    delay_time, leg_ids, control_modes, input_modes, positions, velocities, forces = get_trajectory_from_file(traj_file, frequency)

    for i in range(num_loops):
        print(f"Loop {i+1}/{num_loops}")
        cstart = time.time()
        for i in range(len(delay_time)):

            cnow = time.time()
            while cnow - cstart < delay_time[i]:
                cnow = time.time()

            msg = LegCommand()
            msg.control_mode = control_modes[i]
            msg.pos_setpoint.x = positions[i][0]
            msg.pos_setpoint.y = positions[i][1]
            msg.pos_setpoint.z = positions[i][2]
            msg.vel_setpoint.x = velocities[i][0]
            msg.vel_setpoint.y = velocities[i][1]
            msg.vel_setpoint.z = velocities[i][2]
            msg.force_setpoint.x = forces[i][0]
            msg.force_setpoint.y = forces[i][1]
            msg.force_setpoint.z = forces[i][2]

            publishers[leg_ids[i]].publish(msg)