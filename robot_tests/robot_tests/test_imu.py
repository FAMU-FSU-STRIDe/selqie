import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
import time

def main(args=None):
    rclpy.init(args=args)
    
    node = Node('test_imu_node')
    last_received_time = time.time()
    timeout_threshold = 2.0  # Timeout threshold in seconds (e.g., 2000 ms)

    def imu_callback(msg: Imu):
        nonlocal last_received_time
        last_received_time = time.time()  # Update the last received time
        
        # Clear the console and reset cursor position
        sys.stdout.write("\033[H\033[J")  # ANSI escape sequence to clear screen
        sys.stdout.flush()
        
        # Print IMU data in a static format
        print('-- IMU Data --')
        print(' Orientation:')
        print(f'  x: {msg.orientation.x:.5f}')
        print(f'  y: {msg.orientation.y:.5f}')
        print(f'  z: {msg.orientation.z:.5f}')
        print(' Angular Velocity:')
        print(f'  x: {msg.angular_velocity.x:.5f}')
        print(f'  y: {msg.angular_velocity.y:.5f}')
        print(f'  z: {msg.angular_velocity.z:.5f}')
        print(' Linear Acceleration:')
        print(f'  x: {msg.linear_acceleration.x:.5f}')
        print(f'  y: {msg.linear_acceleration.y:.5f}')
        print(f'  z: {msg.linear_acceleration.z:.5f}')

    def check_timeout():
        nonlocal last_received_time
        current_time = time.time()
        if current_time - last_received_time > timeout_threshold:
            # Clear the console and reset cursor position
            sys.stdout.write("\033[H\033[J")  # ANSI escape sequence to clear screen
            sys.stdout.flush()
            print("IMU not connected or no data received!")
    
    # Create a timer to check for timeout periodically
    timer_period = 0.5  # Check every 0.5 seconds
    node.create_timer(timer_period, check_timeout)
    
    imu_sub = node.create_subscription(Imu, '/imu/data', imu_callback, 10)
    
    rclpy.spin(node)
    rclpy.shutdown()
