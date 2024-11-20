import rclpy
from rclpy.node import Node
from cmd import Cmd
import os
import threading
import subprocess

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from robot_msgs.msg import *
from robot_utils.utils.ros_util_functions import *
from robot_utils.utils.motor_util_functions import *
from robot_utils.utils.leg_util_functions import *
from robot_utils.utils.gait_util_functions import *

NUM_MOTORS = 8
LEG_NAMES = ['FL', 'RL', 'RR', 'FR']
DEFAULT_LEG_POSITION = [0.0, 0.0, -0.18914]
TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')
ROSBAG_RECORD_TOPICS = ['/legFR/command', '/legFL/command', '/legRR/command', '/legRL/command',
                        '/legFR/estimate', '/legFL/estimate', '/legRR/estimate', '/legRL/estimate',
                        '/odrive0/info', '/odrive1/info', '/odrive2/info', '/odrive3/info',
                        '/odrive4/info', '/odrive5/info', '/odrive6/info', '/odrive7/info',]
STAND_HEIGHT = 0.18914
DEFAULT_WALK_FREQUENCY = 2.5

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
            self.motor_command_publishers.append(self.create_publisher(MotorCommand, f'odrive{i}/command', qos_fast()))

        self.motor_estimates = [MotorEstimate() for _ in range(NUM_MOTORS)]
        self.motor_estimate_subscribers = []
        for i in range(NUM_MOTORS):
            motor_estimate_callback = lambda msg, i=i: self.motor_estimates.__setitem__(i, msg)
            self.motor_estimate_subscribers.append(self.create_subscription(MotorEstimate, f'odrive{i}/estimate', motor_estimate_callback, qos_fast()))

        self.motor_config_publishers = []
        for i in range(NUM_MOTORS):
            self.motor_config_publishers.append(self.create_publisher(ODriveConfig, f'odrive{i}/config', qos_reliable()))

        self.motor_infos = [ODriveInfo() for _ in range(NUM_MOTORS)]
        self.motor_info_subscribers = []
        for i in range(NUM_MOTORS):
            motor_info_callback = lambda msg, i=i: self.motor_infos.__setitem__(i, msg)
            self.motor_info_subscribers.append(self.create_subscription(ODriveInfo, f'odrive{i}/info', motor_info_callback, qos_fast()))

        self.cmd_pose_pub = self.create_publisher(Pose, 'cmd_pose', qos_reliable())
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable())
        
        self.odom = Odometry()
        odom_callback = lambda msg: setattr(self, 'odom', msg)
        self.odom_sub = self.create_subscription(Odometry, 'odom', odom_callback, qos_reliable())

        self.stance_pattern_pub = self.create_publisher(StancePattern, 'stance_pattern', qos_reliable())

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

    def do_ready(self, line):
        """
        Ready the ODrive motors
        """
        set_motor_states(self.robot.motor_config_publishers, ODriveConfig.AXIS_STATE_CLOSED_LOOP_CONTROL)

    def do_idle(self, line):
        """
        Idle the ODrive motors
        """
        set_motor_states(self.robot.motor_config_publishers, ODriveConfig.AXIS_STATE_IDLE)

    def do_clear_errors(self, line):
        """
        Clear errors on the ODrive motors
        """
        clear_motor_errors(self.robot.motor_config_publishers)

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

    def do_default(self, line):
        """
        Stand the robot
        """
        set_leg_states(self.robot.leg_command_publishers, MotorCommand.CONTROL_MODE_POSITION, DEFAULT_LEG_POSITION)

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

    def do_print_errors(self, line):
        """
        Print all motor errors
        """
        iserr = False
        for i in range(NUM_MOTORS):
            if self.robot.motor_infos[i].axis_error != 0:
                iserr = True
                print(f"Error on Motor {i}: " + get_error_name(self.robot.motor_infos[i]))
        if not iserr:
            print("No errors on all motors")

    def do_start_recording(self, line):
        """
        Start rosbag recording of specific topics
        Usage: start_recording <output_folder (default: rosbag2_latest)>
        """
        output_folder = "rosbag2_latest"
        if len(line.split()) > 0:
            output_folder = line.split()[0]
            
        if self.rosbag_process is not None:
            print("Rosbag recording is already running")
            return
        
        self.rosbag_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', output_folder] + ROSBAG_RECORD_TOPICS, 
                              stdin=subprocess.DEVNULL)

    def do_stop_recording(self, line):
        """
        Stop rosbag recording
        """
        if self.rosbag_process is None:
            print("Rosbag recording is not running")
            return
        
        self.rosbag_process.terminate()
        self.rosbag_process.wait()
        self.rosbag_process = None

    def do_cmd_vel(self, line):
        """
        Command velocity to the motors
        Usage: cmd_vel <vel_x> <vel_y> <vel_z>
        """
        args = line.split()
        if len(args) != 3:
            print("Invalid number of arguments")
            return
        
        try:
            vel = [float(args[0]), float(args[1]), float(args[2])]
        except ValueError:
            print("Invalid velocity value")
            return
        
        set_cmd_vel(self.robot.cmd_vel_pub, vel)

    def do_cmd_pose(self, line):
        """
        Command velocity to the motors
        Usage: cmd_pose <pos_x> <pos_y> <pos_z> <rot_x> <rot_y> <rot_z>
        """
        args = line.split()
        if len(args) != 6:
            print("Invalid number of arguments")
            return

        try:
            pose = [float(args[0]), float(args[1]), float(args[2])]
            rot = [float(args[3]), float(args[4]), float(args[5])]
        except ValueError:
            print("Invalid position or rotation values")
            return
        
        set_cmd_pose(self.robot.cmd_pose_pub, pose, rot)

    def do_stop_mpc(self, line):
        """
        Stop the MPC controller
        """
        set_cmd_pose(self.robot.cmd_pose_pub, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def do_stand(self, line):
        """
        Stand the robot using MPC
        """
        pose, rot = get_pose(self.robot.odom.pose.pose)
        pose[2] = STAND_HEIGHT
        rot[0:2] = [0.0, 0.0]
        set_cmd_pose(self.robot.cmd_pose_pub, pose, rot)
        set_stand_stance_pattern(self.robot.stance_pattern_pub, self.robot.get_clock())
        
    def do_walk(self, line):
        """
        Walk the robot using MPC
        Usage: walk <frequency (optional)>
        """
        args = line.split()
        if len(args) > 1:
            print("Invalid number of arguments")
            return
        elif len(args) == 0:
            frequency = DEFAULT_WALK_FREQUENCY
        else:
            try:
                frequency = float(args[0])
            except ValueError:
                print("Invalid frequency value")
                return
        
        set_cmd_vel(self.robot.cmd_vel_pub, [0.0, 0.0, 0.0])
        set_walk_stance_pattern(self.robot.stance_pattern_pub, self.robot.get_clock(), frequency)

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
