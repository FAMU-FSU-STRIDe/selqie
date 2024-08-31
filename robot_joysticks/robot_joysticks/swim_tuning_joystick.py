import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from robot_msgs.msg import LegCommand
from robot_msgs.msg import MotorConfig

class SwimTuningJoystick(Node):
    def __init__(self):
        super().__init__('swim_tuning_joystick')

        self.num_motors = 8
        self.declare_parameter('num_motors', self.num_motors)
        self.num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.declare_parameter('leg_names', self.leg_names)
        self.leg_names = self.get_parameter('leg_names').get_parameter_value().string_array_value

        self.motor_config_pubs = []
        for m in range(self.num_motors):
            self.motor_config_pubs.append(self.create_publisher(MotorConfig, f'motor{m}/config', 10))

        self.leg_command_pubs = []
        for l in self.leg_names:
            self.leg_command_pubs.append(self.create_publisher(LegCommand, f'leg{l}/command', 10))

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg : Joy):
        cmd = LegCommand()
        cfg = MotorConfig()

        if msg.buttons[0] == 1:
            # 1
            for i in range(self.num_motors):
                cfg.axis_state = MotorConfig.AXIS_STATE_CLOSED_LOOP_CONTROL
                self.motor_config_pubs[i].publish(cfg)
        if msg.buttons[1] == 1:
            # 2
            for i in range(self.num_motors):
                cfg.axis_state = MotorConfig.AXIS_STATE_IDLE
                self.motor_config_pubs[i].publish(cfg)


def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()