import numpy as np
from dataclasses import dataclass
from robot_msgs.msg import *
from robot_utils.utils.robot_util_functions import *

@dataclass
class SwimTrajectory:
    size = 100
    frequency = 3.5
    L0 = 0.18
    phi = -6*np.pi/8
    xamp = 0.1
    yamp = 0.0
    
def get_swimming_leg_position(traj : SwimTrajectory, i : int) -> list[float]:
    p0 = np.array([traj.L0 * np.cos(traj.phi), 
                   traj.L0 * np.sin(traj.phi)])
        
    theta = traj.phi + np.pi / 2
    rot = np.array([[np.cos(theta), -np.sin(theta)], 
                    [np.sin(theta), np.cos(theta)]])
    
    pr = np.array([traj.xamp * np.cos(2 * np.pi * i / traj.size), 
                   traj.yamp * np.sin(2 * np.pi * i / traj.size)])
    
    p = p0 + np.matmul(rot, pr)

    return [p[0], 0.0, p[1]]

def get_swimming_trajectory(traj : SwimTrajectory, num_legs : int = 4) -> list[LegTrajectory]:
    leg_trajectories = [LegTrajectory() for _ in range(num_legs)]
    for i in range(traj.size):
        for leg_id in range(num_legs):
            leg_trajectories[leg_id].timing.append(i / traj.frequency)
            leg_trajectories[leg_id].commands.append(to_leg_command(3, get_swimming_leg_position(traj, i)))
    return leg_trajectories
