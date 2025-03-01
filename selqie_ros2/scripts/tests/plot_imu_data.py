#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R  # For quaternion operations

class IMUPlotter(Node):

    def __init__(self):
        super().__init__('imu_plotter')
        
        # Create a subscriber to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)
        
        # Deques for storing transformed acceleration data
        self.accel_x_map = deque(maxlen=10000)
        self.accel_y_map = deque(maxlen=10000)
        self.accel_z_map = deque(maxlen=10000)
        self.time = deque(maxlen=10000)
        
        # Plotting setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line_x, = self.ax.plot([], [], label='X')
        self.line_y, = self.ax.plot([], [], label='Y')
        self.line_z, = self.ax.plot([], [], label='Z')
        
        self.ax.legend(prop={'family': 'serif', 'size': 18})  # Set legend font size and family
        self.ax.set_xlabel('Time [s]', fontsize=24, fontdict={'family': 'serif'})
        self.ax.set_ylabel('Acceleration [m/s²]', fontsize=24, fontdict={'family': 'serif'})
        # self.ax.set_title('IMU Acceleration Data', fontsize=28, fontdict={'family': 'serif'})
        self.ax.tick_params(axis='both', which='major', labelsize=18)
        self.ax.grid()
        
        # Start time for relative time calculation
        self.start_time = None
        
        # Call the plotting function periodically
        self.timer = self.create_timer(0.1, self.update_plot)
        self.print_timer = self.create_timer(5.0, self.print_averages)  # Print averages every 5 seconds

    def listener_callback(self, msg : Imu):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1E9
        
        current_time = self.get_clock().now().nanoseconds / 1E9 - self.start_time
        
        # Extract the IMU orientation quaternion
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        
        # Convert quaternion to a rotation matrix
        rotation = R.from_quat(quaternion)  # Uses [x, y, z, w] convention
        rotation_matrix = rotation.as_matrix()
        
        # Extract the linear acceleration in the IMU frame
        accel_imu = np.array([msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z])
        
        # Transform acceleration into the map frame
        accel_map = rotation_matrix @ accel_imu  # Matrix multiplication
        
        # Append the transformed data
        self.accel_x_map.append(accel_map[0])
        self.accel_y_map.append(accel_map[1])
        self.accel_z_map.append(accel_map[2])
        self.time.append(current_time)
        
    def update_plot(self):
        x_data = self.accel_x_map - np.mean(self.accel_x_map)
        y_data = self.accel_y_map - np.mean(self.accel_y_map)
        z_data = self.accel_z_map - np.mean(self.accel_z_map)
        
        # Update plot data
        self.line_x.set_data(self.time, x_data)
        self.line_y.set_data(self.time, y_data)
        self.line_z.set_data(self.time, z_data)
        
        # Update plot limits
        if self.time:
            self.ax.set_xlim(min(self.time), max(self.time))
            self.ax.set_ylim(
                min(min(x_data, default=0), 
                    min(y_data, default=0), 
                    min(z_data, default=0)) - 0.025,
                max(max(x_data, default=0), 
                    max(y_data, default=0), 
                    max(z_data, default=0)) + 0.025
            )
        
        # Refresh plot
        plt.draw()
        plt.pause(0.01)

    def print_averages(self):
        # Calculate averages of accelerations
        if self.accel_x_map and self.accel_y_map and self.accel_z_map:
            avg_x = np.mean(self.accel_x_map)
            avg_y = np.mean(self.accel_y_map)
            avg_z = np.mean(self.accel_z_map)
            var_x = np.var(self.accel_x_map)
            var_y = np.var(self.accel_y_map)
            var_z = np.var(self.accel_z_map)    
            self.get_logger().info(f"Average Accelerations (map frame): "
                                   f"X: {avg_x}, Y: {avg_y}, Z: {avg_z}")
            self.get_logger().info(f"Variance of Accelerations (map frame): "
                                   f"X: {var_x}, Y: {var_y}, Z: {var_z}")

def main(args=None):
    rclpy.init(args=args)
    imu_plotter = IMUPlotter()
    
    try:
        rclpy.spin(imu_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        imu_plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
