import os
import time
from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import *

TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')

def to_leg_command(mode, pos, vel = [0.0, 0.0, 0.0], force = [0.0, 0.0, 0.0]):
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
    return msg

def set_leg_states(cmd_publishers, mode, pos, vel = [0.0, 0.0, 0.0], force = [0.0, 0.0, 0.0]):
    msg = to_leg_command(mode, pos, vel, force)
    for cmd_pub in cmd_publishers:
        cmd_pub.publish(msg)

def get_trajectories_from_file(rel_file : str, frequency : float, num_legs : int = 4):
    file = os.path.join(TRAJECTORIES_FOLDER, rel_file)
    if not os.path.exists(file):
        print('File ' + file + ' does not exist')
        return
    
    leg_trajectories = [LegTrajectory() for _ in range(num_legs)]
    with open(file) as f:
        for line in f:
            parts = line.split()
            if len(parts) != 13:
                print('Invalid file line: ' + line)
                return
            
            time = float(parts[0]) / 1000.0 / frequency
            leg_id = int(parts[1])
            mode = int(parts[2])
            position = [float(parts[4]), float(parts[5]), float(parts[6])]
            velocity = [float(parts[7]), float(parts[8]), float(parts[9])]
            force = [float(parts[10]), float(parts[11]), float(parts[12])]
            
            if (leg_id > num_legs) or (leg_id < 0):
                print(f'Expected leg ids between 0 and {num_legs - 1}')
                return
            leg_trajectories[leg_id].timing.append(time)
            leg_trajectories[leg_id].commands.append(to_leg_command(mode, position, velocity, force))

    return leg_trajectories

def run_leg_trajectory(traj_publishers : list, trajectories : list, num_loops : int, frequency : float):
    print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
    for i in range(num_loops):
        print(f"  Loop {i+1}/{num_loops}")
        for j in range(len(traj_publishers)):
            if len(trajectories) == 1:
                traj_publishers[j].publish(trajectories[0])
            else:
                traj_publishers[j].publish(trajectories[j])
        time.sleep(1.0 / frequency)
    print("Finished trajectory")