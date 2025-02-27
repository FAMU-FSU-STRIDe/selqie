#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class DepthPlotter(Node):

    def __init__(self):
        super().__init__('depth_plotter')
        
        # Create a subscriber to the depth topic
        self.subscription = self.create_subscription(
            Float32,
            '/bar100/depth',
            self.listener_callback,
            10)
        
        # Deques for storing depth data
        self.depth_map = deque(maxlen=2000)
        self.time = deque(maxlen=2000)
        
        # Plotting setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], linewidth=3)
        
        self.ax.legend(prop={'family': 'serif', 'size': 18})  # Set legend font size and family
        self.ax.set_xlabel('Time [s]', fontsize=24, fontdict={'family': 'serif'})
        self.ax.set_ylabel('Depth [m]', fontsize=24, fontdict={'family': 'serif'})
        # self.ax.set_title('Depth Data Over Time', fontsize=28, fontdict={'family': 'serif'})
        self.ax.tick_params(axis='both', which='major', labelsize=18)
        self.ax.grid()
        
        # Start time for relative time calculation
        self.start_time = None
        
        # Call the plotting function periodically
        self.timer = self.create_timer(0.1, self.update_plot)
        self.print_timer = self.create_timer(5.0, self.print_averages)  # Print averages every 5 seconds

    def listener_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1E9
            
        current_time = self.get_clock().now().nanoseconds / 1E9 - self.start_time
        depth = msg.data
        self.depth_map.append(depth)
        self.time.append(current_time)
        
    def update_plot(self):
        depth_data = self.depth_map - np.mean(self.depth_map)
        
        # Update plot data
        self.line.set_data(self.time, depth_data)
        
        # Update plot limits
        if self.time:
            self.ax.set_xlim(min(self.time), max(self.time))
            self.ax.set_ylim(min(depth_data) - 0.05, max(depth_data) + 0.05)
        
        # Refresh plot
        plt.draw()
        plt.pause(0.01)
        
    def print_averages(self):
        # Calculate averages of accelerations
        if self.depth_map:
            avg_depth = np.mean(self.depth_map)
            var_depth = np.var(self.depth_map)    
            self.get_logger().info(f"Average: {avg_depth}")
            self.get_logger().info(f"Variance: {var_depth}")

def main(args=None):
    rclpy.init(args=args)
    depth_plotter = DepthPlotter()
    
    try:
        rclpy.spin(depth_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        depth_plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
