import os
import rclpy
import time
from rclpy.node import Node
from robot_msgs.msg import LegCommand
from ament_index_python.packages import get_package_share_directory

trajectories_folder = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')

def main(args=None):
    rclpy.init(args=args)

    node = Node('run_leg_trajectory_file')

    node.declare_parameter('file', "walk.txt")
    file = node.get_parameter('file').get_parameter_value().string_value

    node.declare_parameter('num_loops', 1)
    num_loops = node.get_parameter('num_loops').get_parameter_value().integer_value

    node.declare_parameter('frequency', 1.0)
    frequency = node.get_parameter('frequency').get_parameter_value().double_value

    node.declare_parameter('leg_names', ['FL', 'FR', 'RL', 'RR'])
    leg_names = node.get_parameter('leg_names').get_parameter_value().string_array_value

    node.get_logger().info("Connecting to Legs...")

    publishers = []
    for i in range(4):
        publishers.append(node.create_publisher(LegCommand, f'leg{leg_names[i]}/command', 10))
        
        # while publishers[i].get_subscription_count() == 0:
        #    time.sleep(0.1)
            
    node.get_logger().info("Connected to Legs")

    delay_time = []
    leg_ids = []
    control_modes = []
    input_modes = [] # not used
    positions = []
    velocities = []
    forces = []

    with open(os.path.join(trajectories_folder, file)) as f:
        for line in f:
            parts = line.split()

            if len(parts) != 13:
                node.get_logger().error('Invalid file line: ' + line)
                return
            
            delay_time.append(float(parts[0]) / 1000.0 / frequency)
            leg_ids.append(int(parts[1]))
            control_modes.append(int(parts[2]))
            input_modes.append(int(parts[3]))
            positions.append([float(parts[4]), float(parts[5]), float(parts[6])])
            velocities.append([float(parts[7]), float(parts[8]), float(parts[9])])
            forces.append([float(parts[10]), float(parts[11]), float(parts[12])])

    if (max(leg_ids) > len(leg_names)) or (min(leg_ids) < 0):
        node.get_logger().error(f'Expected {len(leg_names)} leg ids')
        return
    
    # Sort by delay time
    delay_time, leg_ids, control_modes, input_modes, positions, velocities, forces = zip(
        *sorted(zip(delay_time, leg_ids, control_modes, input_modes, positions, velocities, forces))
    )

    node.get_logger().info(f"Running Trajectory from file {file}")
    for i in range(num_loops):
        node.get_logger().info(f"Loop {i+1}/{num_loops}")
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

    node.destroy_node()
    rclpy.shutdown()