import rclpy
from rclpy.node import Node
from cmd import Cmd
import os
import threading
import subprocess

from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import *
from robot_utils.utils.ros_util_functions import *
from robot_utils.utils.motor_util_functions import *
from robot_utils.utils.leg_util_functions import *

NUM_MOTORS = 8
LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
STANDING_LEG_POSITION = [0.0, 0.0, -0.18914]
TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')

class SELQIERobotNode(Node):
    def __init__(self):
        super().__init__('robot_terminal_node')
        
        print("Connecting...")
        
        self.leg_command_publishers = []
        for i in range(len(LEG_NAMES)):
            self.leg_command_publishers.append(self.create_publisher(LegCommand, f'leg{LEG_NAMES[i]}/command', qos_fast()))

        self.leg_estimates = [LegEstimate() for _ in range(len(LEG_NAMES))]
        self.leg_estimate_subscribers = []
        for i in range(len(LEG_NAMES)):
            leg_estimate_callback = lambda msg, i=i: self.leg_estimates.__setitem__(i, msg)
            self.leg_estimate_subscribers.append(self.create_subscription(LegEstimate, f'leg{LEG_NAMES[i]}/estimate', leg_estimate_callback, qos_fast()))

        self.leg_trajectory_publishers = []
        for i in range(len(LEG_NAMES)):
            self.leg_trajectory_publishers.append(self.create_publisher(LegTrajectory, f'leg{LEG_NAMES[i]}/trajectory', qos_reliable()))

        self.motor_command_publishers = []
        for i in range(NUM_MOTORS):
            self.motor_command_publishers.append(self.create_publisher(MotorCommand, f'motor{i}/command', qos_fast()))

        self.motor_estimates = [MotorEstimate() for _ in range(NUM_MOTORS)]
        self.motor_estimate_subscribers = []
        for i in range(NUM_MOTORS):
            motor_estimate_callback = lambda msg, i=i: self.motor_estimates.__setitem__(i, msg)
            self.motor_estimate_subscribers.append(self.create_subscription(MotorEstimate, f'motor{i}/estimate', motor_estimate_callback, qos_fast()))

        self.motor_config_publishers = []
        for i in range(NUM_MOTORS):
            self.motor_config_publishers.append(self.create_publisher(ODriveConfig, f'motor{i}/config', qos_reliable()))

        if not wait_for_subs(self.leg_command_publishers):
            print("Failed to connect to Leg command")
        if not wait_for_subs(self.motor_command_publishers):
            print("Failed to connect to Motor command")
        if not wait_for_subs(self.motor_config_publishers):
            print("Failed to connect to Motor config")

class STARQTerminal(Cmd):
    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '
    
    def __init__(self, robot_instance : SELQIERobotNode):
        super().__init__()
        self.robot = robot_instance

    def do_exit(self, line):
        """
        Exit the terminal.
        """
        print("Exiting...")
        rclpy.shutdown()
        return True

    def do_zero(self, line):
        """
        Zero the motors
        """
        set_motor_positions(self.robot.motor_command_publishers, 0.0)

    def do_set_motor_position(self, line):
        """
        Set the position of a motor
        """
        args = line.split()
        if len(args) != 2:
            print("Invalid number of arguments")
            return
        
        try:
            motor = int(args[0])
            position = float(args[1])
        except ValueError:
            print("Invalid motor or position values")
            return
        
        set_motor_positions([self.robot.motor_command_publishers[motor]], position)

    def do_set_gains(self, line):
        """
        Set the gains for the motors
        Usage: set_gains <p_gain> <v_gain> <vi_gain>
        """
        args = line.split()
        if len(args) != 3:
            print("Invalid number of arguments")
            return

        try:
            gains = [float(args[0]), float(args[1]), float(args[2])]
        except ValueError:
            print("Invalid gain values")
            return
        
        set_motor_gains(self.robot.motor_config_publishers, gains)

    def do_stand(self, line):
        """
        Stand the robot
        """
        set_leg_states(self.robot.leg_command_publishers, MotorCommand.CONTROL_MODE_POSITION, STANDING_LEG_POSITION)

    def do_set_leg_position(self, line):
        """
        Set the position of a leg
        Usage: set_leg_position <leg_name> <x> <y> <z>
        Note: Use leg name '*' to set position for all legs
        """
        args = line.split()
        if len(args) != 4:
            print("Invalid number of arguments")
            return
        
        leg = args[0]
        if leg == "*":
            cmd_publishers = self.robot.leg_command_publishers
        elif leg in LEG_NAMES:
            cmd_publishers = [self.robot.leg_command_publishers[LEG_NAMES.index(leg)]]
        else:
            print("Invalid leg name")
            return
    
        try:
            pos = [float(args[1]), float(args[2]), float(args[3])]
        except ValueError:
            print("Invalid position values")
            return
        
        set_leg_states(cmd_publishers, MotorCommand.CONTROL_MODE_POSITION, pos)
    
    def do_run_trajectory(self, line):
        """
        Run a trajectory file or sequence of files
        Usage: run_trajectory <file1> <num_loops1> <frequency1> <file2> <num_loops2> <frequency2> ...
        """
        args = line.split()
        if len(args) % 3 != 0:
            print("Invalid number of arguments")
            return
        
        for i in range(0, len(args), 3):
            file = args[0]
            try:
                num_loops = int(args[i+1])
                frequency = float(args[i+2])
            except ValueError:
                print("Invalid number of loops or frequency")
                return

            trajectories = get_trajectories_from_file(file, frequency)
            run_leg_trajectory(self.robot.leg_trajectory_publishers, trajectories, num_loops, frequency)
            
    def complete_run_trajectory(self, text, line, begidx, endidx):
        if len(line.split()) % 3 == 1 or len(line.split()) % 3 == 2:
            files = os.listdir(TRAJECTORIES_FOLDER)
            return [f for f in files if f.startswith(text)]
        return []

    def do_print_motor_info(self, line):
        """
        Print motor info
        """
        for i in range(NUM_MOTORS):
            print(f"Motor {i}:")
            print_motor_estimate(self.robot.motor_estimates[i])

    def do_print_leg_info(self, line):
        """
        Print leg info
        """
        for i in range(len(LEG_NAMES)):
            print(f"Leg {LEG_NAMES[i]}:")
            print_leg_info(self.robot.leg_estimates[i])

def main():
    rclpy.init()
    robot = SELQIERobotNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(robot,))
    spin_thread.start()
    STARQTerminal(robot).cmdloop()
    spin_thread.join()
    robot.destroy_node()

if __name__ == '__main__':
    main()
