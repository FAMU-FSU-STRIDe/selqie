import rclpy
from rclpy.node import Node
from cmd import Cmd
import os
import time
import threading
import subprocess

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo
from robot_utils.utils.robot_util_functions import set_leg_states, run_leg_trajectory_file, \
                                             print_motor_info, print_leg_info, get_error_name
from robot_utils.utils.odrive_util_functions import set_odrive_states, clear_odrive_errors, \
                                             set_odrive_positions, set_odrive_gains

NUM_MOTORS = 8
LEG_NAMES = ['FL', 'FR', 'RL', 'RR']
STANDING_LEG_POSITION = [0.0, 0.0, -0.18914]
TRAJECTORIES_FOLDER = os.path.join(get_package_share_directory('robot_utils'), 'trajectories')
ROSBAG_RECORD_TOPICS = ['/legFR/command', '/legFL/command', '/legRR/command', '/legRL/command',
                        '/legFR/estimate', '/legFL/estimate', '/legRR/estimate', '/legRL/estimate',
                        '/odrive0/info', '/odrive1/info', '/odrive2/info', '/odrive3/info',
                        '/odrive4/info', '/odrive5/info', '/odrive6/info', '/odrive7/info',]
JOYSTICK_PACKAGE = 'robot_joysticks'

def qos_fast():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

def qos_reliable():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        depth=10
    )
    
def wait_for_subs(pubs, timeout=5):
    cstart = time.time()
    while not all(pub.get_subscription_count() != 0 for pub in pubs):
        if time.time() - cstart > timeout:
            return False
        time.sleep(0.05)
    return True

class STARQRobotNode(Node):
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
            self.motor_config_publishers.append(self.create_publisher(MotorConfig, f'odrive{i}/config', qos_reliable()))

        self.motor_infos = [MotorInfo() for _ in range(NUM_MOTORS)]
        self.motor_info_subscribers = []
        for i in range(NUM_MOTORS):
            motor_info_callback = lambda msg, i=i: self.motor_infos.__setitem__(i, msg)
            self.motor_info_subscribers.append(self.create_subscription(MotorInfo, f'odrive{i}/info', motor_info_callback, qos_fast()))

        if not wait_for_subs(self.leg_command_publishers):
            print("Failed to connect to Leg command")
        if not wait_for_subs(self.motor_command_publishers):
            print("Failed to connect to Motor command")
        if not wait_for_subs(self.motor_config_publishers):
            print("Failed to connect to Motor config")
        
    def motor_callback(self, msg):
        self.motor_status = msg.data

    def get_status(self):
        return f"Motor Status: {self.motor_status}, Leg Status: {self.leg_status}"

class STARQTerminal(Cmd):
    intro = 'Welcome to the STARQ terminal. Type help or ? to list commands.\n'
    prompt = 'STARQ> '
    
    def __init__(self, robot_instance : STARQRobotNode):
        super().__init__()
        self.robot = robot_instance
        self.rosbag_process = None
        self.joystick_process = None

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
        set_odrive_states(self.robot.motor_config_publishers, MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL)

    def do_idle(self, line):
        """
        Idle the ODrive motors
        """
        set_odrive_states(self.robot.motor_config_publishers, MotorConfig.AXIS_STATE_IDLE)

    def do_clear_errors(self, line):
        """
        Clear errors on the ODrive motors
        """
        clear_odrive_errors(self.robot.motor_config_publishers)

    def do_zero(self, line):
        """
        Zero the ODrive motors
        """
        set_odrive_positions(self.robot.motor_command_publishers, 0.0)

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
        
        set_odrive_gains(self.robot.motor_config_publishers, gains)

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
            file = os.path.join(TRAJECTORIES_FOLDER, args[i])
            if not os.path.exists(file):
                print('File ' + file + ' does not exist')
                return
        
            try:
                num_loops = int(args[i+1])
                frequency = float(args[i+2])
            except ValueError:
                print("Invalid number of loops or frequency")
                return

            run_leg_trajectory_file(self.robot.leg_command_publishers, file, num_loops, frequency)
            
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
            print_motor_info(self.robot.motor_infos[i], self.robot.motor_estimates[i])

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

def main():
    rclpy.init()
    robot = STARQRobotNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(robot,))
    spin_thread.start()
    STARQTerminal(robot).cmdloop()
    spin_thread.join()
    robot.destroy_node()

if __name__ == '__main__':
    main()
