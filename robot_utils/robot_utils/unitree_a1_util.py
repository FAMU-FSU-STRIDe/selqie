import rclpy
from rclpy.node import Node
from cmd import Cmd
import os
import threading

from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import *
from robot_utils.utils.robot_util_functions import *
from geometry_msgs.msg import Pose, Twist

NUM_MOTORS = 12
LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
LEFT_STANDING_LEG_POSITION = [0.0, 0.08505, -0.27]
RIGHT_STANDING_LEG_POSITION = [0.0, -0.08505, -0.27]
TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')
JOYSTICK_PACKAGE = 'robot_joysticks'

class UnitreeA1RobotNode(Node):
    def __init__(self):
        super().__init__('unitree_terminal_node')
        
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

        self.cmd_pose_pub = self.create_publisher(Pose, 'cmd_pose', qos_reliable())
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())

        self.stance_pattern_pub = self.create_publisher(StancePattern, 'stance_pattern', qos_reliable())

        if not wait_for_subs(self.leg_command_publishers):
            print("Failed to connect to Leg command")
        if not wait_for_subs(self.motor_command_publishers):
            print("Failed to connect to Motor command")
        if not wait_for_subs(self.motor_config_publishers):
            print("Failed to connect to Motor config")

class UnitreeA1Terminal(Cmd):
    intro = 'Welcome to the Unitree A1 terminal. Type help or ? to list commands.\n'
    prompt = 'A1> '
    
    def __init__(self, robot_instance : UnitreeA1RobotNode):
        super().__init__()
        self.robot = robot_instance
        self.rosbag_process = None
        self.leg_trajectory_process = None

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

    def do_set_gains(self, line):
        """
        Set the gains for the ODrive motors
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
        set_leg_states(self.robot.leg_command_publishers[0:2], MotorCommand.CONTROL_MODE_POSITION, LEFT_STANDING_LEG_POSITION)
        set_leg_states(self.robot.leg_command_publishers[2:4], MotorCommand.CONTROL_MODE_POSITION, RIGHT_STANDING_LEG_POSITION)

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

    def do_print_motor_estimate(self, line):
        """
        Print motor estimate
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

    def do_cmd_vel(self, line):
        """
        Command velocity to the motors
        Usage: cmd_vel <vel_x>
        """
        try:
            velocity = float(line)
        except ValueError:
            print("Invalid velocity value")
            return
        
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = 0.0
        self.robot.cmd_vel_pub.publish(twist)

    def do_cmd_pose(self, line):
        """
        Command velocity to the motors
        Usage: cmd_pose <pos_x> <pos_y> <pos_z> <rot_x> <rot_y> <rot_z>
        """
        args = line.split()
        if len(args) != 6:
            print("Invalid number of arguments")
            return

        pose = Pose()
        try:
            pose.position.x = float(args[0])
            pose.position.y = float(args[1])
            pose.position.z = float(args[2])
            pose.orientation.x = float(args[3])
            pose.orientation.y = float(args[4])
            pose.orientation.z = float(args[5])
        except ValueError:
            print("Invalid position or rotation values")
            return
        
        self.robot.cmd_pose_pub.publish(pose)

    def do_gait(self, line):
        """
        Set the robot MPC gait
        """
        args = line.split()
        if len(args) != 1:
            print("Invalid number of arguments")
            return
        
        gait = args[0]
        if gait == "stand":
            pattern = StancePattern()
            pattern.header.stamp = self.robot.get_clock().now().to_msg()
            pattern.frequency = 1.0
            pattern.timing = [0.0, 1.0]
            pattern.stance = [0b1111, 0b1111]
            self.robot.stance_pattern_pub.publish(pattern)
            self.do_cmd_pose("0.0 0.0 0.27 0.0 0.0 0.0")
        else:
            print("Invalid gait name")
            return
        
        

def main():
    rclpy.init()
    robot = UnitreeA1RobotNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(robot,))
    spin_thread.start()
    UnitreeA1Terminal(robot).cmdloop()
    spin_thread.join()
    robot.destroy_node()

if __name__ == '__main__':
    main()
