import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class GPIONode(Node):
    def __init__(self):
        super().__init__('gpio_node')

        self.declare_parameter('gpio_pin', 18)
        self.gpio_pin = self.get_parameter('gpio_pin').value

        self.declare_parameter('is_output', True)
        self.is_output = self.get_parameter('is_output').value

        self.declare_parameter('is_pwm', False)
        self.is_pwm = self.get_parameter('is_pwm').value

        self.declare_parameter('frequency', 50.0)
        self.frequency = self.get_parameter('frequency').value

        self.declare_parameter('initial_value', GPIO.LOW)
        self.initial_value = self.get_parameter('initial_value').value

        GPIO.setmode(GPIO.BOARD)

        if self.is_output or self.is_pwm:
            GPIO.setup(self.gpio_pin, GPIO.OUT, initial=self.initial_value)
            self.subscriber = self.create_subscription(UInt8, 'gpio/out', self.subscriber_callback, 10)
        else:
            GPIO.setup(self.gpio_pin, GPIO.IN)
            self.publisher = self.create_publisher(UInt8, 'gpio/in', 10)
            self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
            
        if self.is_pwm:
            self.pwm = GPIO.PWM(self.gpio_pin, self.frequency)
            self.pwm.start(self.initial_value)
            

    def on_cleanup(self):
        if self.is_pwm:
            self.pwm.stop()
        elif self.is_output:
            GPIO.output(self.gpio_pin, GPIO.LOW)
        GPIO.cleanup()

    def timer_callback(self):
        val = GPIO.input(self.gpio_pin)
        msg = UInt8()
        msg.data = val
        self.publisher.publish(msg)

    def subscriber_callback(self, msg):
        val = msg.data
        if self.is_pwm:
            if val > 100:
                val = 100
            self.pwm.ChangeDutyCycle(val)
        elif val == 0:
            GPIO.output(self.gpio_pin, GPIO.LOW)
        else:
            GPIO.output(self.gpio_pin, GPIO.HIGH)

def main(args=None):
    rclpy.init(args=args)
    node = GPIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_cleanup()
    node.destroy_node()
    rclpy.shutdown()