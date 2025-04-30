import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import Jetson.GPIO as GPIO # Driver for NVIDIA Jetson GPIO pins

class JetsonGPIONode(Node):
    """
    Node to control GPIO pins on NVIDIA Jetson devices.
    It can be configured to operate in input, output, or PWM mode.
    """
    def __init__(self):
        super().__init__('gpio_node')

        # Get ROS parameters
        self.declare_parameter('gpio_pin', 18)
        self.gpio_pin = self.get_parameter('gpio_pin').value

        self.declare_parameter('is_output', True)
        self.is_output = self.get_parameter('is_output').value

        # Setting PWM to True will automatically set is_output to True
        self.declare_parameter('is_pwm', False)
        self.is_pwm = self.get_parameter('is_pwm').value

        # Frequency is for PWM if enabled, or for input polling if not
        self.declare_parameter('frequency', 50.0)
        self.frequency = self.get_parameter('frequency').value

        self.declare_parameter('initial_value', GPIO.LOW)
        self.initial_value = self.get_parameter('initial_value').value

        # Set GPIO mode
        GPIO.setmode(GPIO.BOARD)

        if self.is_output or self.is_pwm:
            # If in output mode or the PWM is enabled, set the GPIO pin as output
            GPIO.setup(self.gpio_pin, GPIO.OUT, initial=self.initial_value)
            
            # Create subscriber for PWM or GPIO output
            self.subscriber = self.create_subscription(Float32, 'gpio/out', self.subscriber_callback, 10)
        else:
            # Otherwise, set the GPIO pin as input
            GPIO.setup(self.gpio_pin, GPIO.IN)
            
            # Create publisher for GPIO input
            self.publisher = self.create_publisher(Float32, 'gpio/in', 10)
            
            # Create polling timer
            self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
            
        if self.is_pwm:
            # If in PWM mode, set up PWM
            self.pwm = GPIO.PWM(self.gpio_pin, self.frequency)
            self.pwm.start(self.initial_value)
            
        self.get_logger().info(f"Jetson GPIO Node Initialized on pin {self.gpio_pin} with mode "
                               f"{'PWM' if self.is_pwm else 'Output' if self.is_output else 'Input'}")
            

    def on_cleanup(self):
        """Clean up GPIO settings on shutdown."""
        if self.is_pwm:
            # Stop PWM if it was started
            self.pwm.stop()
        elif self.is_output:
            # Set GPIO pin to LOW if in output mode
            GPIO.output(self.gpio_pin, GPIO.LOW)
            
        # Clean up GPIO settings
        GPIO.cleanup()
        
        self.get_logger().info("Jetson GPIO Node shut down.")

    def timer_callback(self):
        """Callback for the timer to read GPIO input."""
        # Read GPIO input and publish the value
        val = GPIO.input(self.gpio_pin)
        msg = Float32()
        msg.data = val
        self.publisher.publish(msg)

    def subscriber_callback(self, msg : Float32):
        """Callback for the subscriber to set GPIO output or PWM duty cycle."""
        val = msg.data
        
        if self.is_pwm:
            # If in PWM mode, set the duty cycle
            if val > 100.0:
                # Clamp the value to 100% if it exceeds
                val = 100.0
            self.pwm.ChangeDutyCycle(val)
            # User feedback
            self.get_logger().info(f"Set PWM duty cycle on {self.gpio_pin} to {val}%")
        elif val == 0:
            # If the value is 0, set GPIO pin to LOW
            GPIO.output(self.gpio_pin, GPIO.LOW)
            # User feedback
            self.get_logger().info(f"Set GPIO pin {self.gpio_pin} to LOW")
        else:
            # Otherwise, set GPIO pin to HIGH
            GPIO.output(self.gpio_pin, GPIO.HIGH)
            # User feedback
            self.get_logger().info(f"Set GPIO pin {self.gpio_pin} to HIGH")

# Entry point for the node
def main(args=None):
    rclpy.init(args=args)
    node = JetsonGPIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        node.on_cleanup()
    node.destroy_node()
    rclpy.shutdown()