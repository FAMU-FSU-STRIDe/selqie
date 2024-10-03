import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

class GPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')

        self.declare_parameter('gpio_pin', 18)
        self.gpio_pin = self.get_parameter('gpio_pin').value

        self.declare_parameter('is_input', False)
        self.is_input = self.get_parameter('is_input').value

        self.declare_parameter('is_pwm', True)
        self.is_pwm = self.get_parameter('is_pwm').value

        GPIO.setmode(GPIO.BOARD)
        self.get_logger().info('GPIO Mode: BOARD')
        self.get_logger().info('Model: %s' % GPIO.model)

        self.get_logger().info('Pin: %d' % self.gpio_pin)

        if self.is_input:
            GPIO.setup(self.gpio_pin, GPIO.IN)
            self.get_logger().info('Mode: INPUT')
        else:
            GPIO.setup(self.gpio_pin, GPIO.OUT, initial=GPIO.LOW)
            self.get_logger().info('Mode: OUTPUT')

        if self.is_pwm:
            self.pwm = GPIO.PWM(self.gpio_pin, 50)
            self.pwm.start(50)
            self.get_logger().info('PWM: True')
        else:
            GPIO.output(self.gpio_pin, GPIO.LOW)
            self.get_logger().info('PWM: False')

    def __del__(self):
        self.pwm.stop()
        GPIO.output(self.gpio_pin, GPIO.LOW)
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = GPIONode()
    rclpy.spin(node)
    rclpy.shutdown()