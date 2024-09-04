import rclpy
from rclpy.node import Node
from cmd import Cmd

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from robot_msgs.msg import LegCommand, LegEstimate, MotorCommand, MotorEstimate, MotorConfig, MotorInfo
from robot_utils.robot_util_functions import run_leg_trajectory_file

NUM_MOTORS = 8
LEG_NAMES = ['FL', 'FR', 'RL', 'RR']
STANDING_LEG_POSITION = [0.0, 0.0, -0.18914]

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

class STARQRobotNode(Node):
    def __init__(self):
        super().__init__('robot_terminal_node')
        
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

    def motor_callback(self, msg):
        self.motor_status = msg.data

    def get_status(self):
        return f"Motor Status: {self.motor_status}, Leg Status: {self.leg_status}"

class STARQTerminal(Cmd):
    intro = 'Welcome to the STARQ terminal. Type help or ? to list commands.\n'
    prompt = 'STARQ> '
    
    def __init__(self, robot_instance):
        super().__init__()
        self.robot = robot_instance

    def do_exit(self, line):
        """Exit the terminal."""
        print("Exiting...")
        return True

    def do_ready(self, line):
        """Ready the ODrive motors"""
        msg = MotorConfig()
        msg.axis_state = MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL
        for cfg_pub in self.robot.motor_config_publishers:
            cfg_pub.publish(msg)

    def do_idle(self, line):
        """Idle the ODrive motors"""
        msg = MotorConfig()
        msg.axis_state = MotorConfig.AXIS_STATE_IDLE
        for cfg_pub in self.robot.motor_config_publishers:
            cfg_pub.publish(msg)

    def do_clear_errors(self, line):
        """Clear errors on the ODrive motors"""
        msg = MotorConfig()
        msg.clear_errors = True
        for cfg_pub in self.robot.motor_config_publishers:
            cfg_pub.publish(msg)

    def do_zero(self, line):
        """Zero the ODrive motors"""
        msg = MotorCommand()
        msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
        msg.pos_setpoint = 0.0
        for pub in self.robot.motor_command_publishers:
            pub.publish(msg)

    def do_stand(self, line):
        """Stand the robot"""
        msg = LegCommand()
        msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
        msg.pos_setpoint.x = STANDING_LEG_POSITION[0]
        msg.pos_setpoint.y = STANDING_LEG_POSITION[1]
        msg.pos_setpoint.z = STANDING_LEG_POSITION[2]
        for pub in self.robot.leg_command_publishers:
            pub.publish(msg)

    def do_set_leg_position(self, line):
        """Set the position of a leg"""
        args = line.split()
        if len(args) != 4:
            print("Invalid number of arguments")
            return
        
        leg = args[0]
        if leg not in LEG_NAMES:
            print("Invalid leg name")
            return
    
        try:
            x = float(args[1])
            y = float(args[2])
            z = float(args[3])
        except ValueError:
            print("Invalid position values")
            return
        
        msg = LegCommand()
        msg.control_mode = MotorCommand.CONTROL_MODE_POSITION
        msg.pos_setpoint.x = x
        msg.pos_setpoint.y = y
        msg.pos_setpoint.z = z
        self.robot.leg_command_publishers[LEG_NAMES.index(leg)].publish(msg)

    def do_run_trajectory(self, line):
        """Run a trajectory file"""
        args = line.split()
        if len(args) != 3:
            print("Invalid number of arguments")
            return
        
        file = args[0]
        try:
            num_loops = int(args[1])
            frequency = float(args[2])
        except ValueError:
            print("Invalid number of loops or frequency")
            return

        run_leg_trajectory_file(file, num_loops, frequency, self.robot.leg_command_publishers)

    def do_print_motor_info(self, line):
        """Print motor info"""
        for i in range(NUM_MOTORS):
            print(f"Motor {i}: {self.robot.motor_infos[i]}")
            print(f"Motor {i}: {self.robot.motor_estimates[i]}")

    def do_print_leg_info(self, line):
        """Print leg info"""
        for i in range(len(LEG_NAMES)):
            print(f"Leg {LEG_NAMES[i]}: {self.robot.leg_estimates[i]}")

def main():
    rclpy.init()
    robot = STARQRobotNode()
    STARQTerminal(robot).cmdloop()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
