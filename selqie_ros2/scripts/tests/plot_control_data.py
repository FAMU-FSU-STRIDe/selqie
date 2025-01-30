#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque

class ControlPlotter(Node):

    def __init__(self):
        super().__init__('control_plotter')
        
        # Subscribers
        self.latest_odom = Odometry()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Data storage
        self.odom_x = deque(maxlen=2000)
        self.odom_y = deque(maxlen=2000)
        self.cmd_vx = deque(maxlen=2000)
        self.cmd_wz = deque(maxlen=2000)
        self.time = deque(maxlen=2000)
        
        # Plotting setup
        plt.ion()
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 6), 
                                          gridspec_kw={'width_ratios': [2, 1], 'height_ratios': [1, 1]})

        # Left plot: Robot path
        self.axs[0, 0].set_title("Robot Path (X-Y)")
        self.axs[0, 0].set_xlabel("X Position (m)")
        self.axs[0, 0].set_ylabel("Y Position (m)")
        self.axs[0, 0].set_aspect('equal')  # Equal axis scaling
        self.path_line, = self.axs[0, 0].plot([], [], 'b-', label='Path')
        self.axs[0, 0].legend()

        # Top right plot: cmd_vx over time
        self.axs[0, 1].set_title("Commanded Velocity (Vx)")
        self.axs[0, 1].set_xlabel("Time (s)")
        self.axs[0, 1].set_ylabel("Vx (m/s)")
        self.vx_line, = self.axs[0, 1].plot([], [], 'r-', label='cmd_vx')
        self.axs[0, 1].legend()

        # Bottom right plot: cmd_wz over time
        self.axs[1, 1].set_title("Commanded Angular Velocity (Wz)")
        self.axs[1, 1].set_xlabel("Time (s)")
        self.axs[1, 1].set_ylabel("Wz (rad/s)")
        self.wz_line, = self.axs[1, 1].plot([], [], 'g-', label='cmd_wz')
        self.axs[1, 1].legend()

        # Remove empty bottom-left plot
        self.axs[1, 0].axis('off')

        # Start time
        self.start_time = self.get_clock().now().nanoseconds / 1E9

        # Periodic plot update
        self.timer = self.create_timer(0.1, self.update_plot)

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_vel_callback(self, msg: Twist):
        current_time = self.get_clock().now().nanoseconds / 1E9 - self.start_time
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y
        cmd_vx = msg.linear.x
        cmd_wz = msg.angular.z

        self.odom_x.append(odom_x)
        self.odom_y.append(odom_y)
        self.cmd_vx.append(cmd_vx)
        self.cmd_wz.append(cmd_wz)
        self.time.append(current_time)

    def update_plot(self):
        if not self.time:
            return

        # Update robot path
        self.path_line.set_data(self.odom_x, self.odom_y)
        self.axs[0, 0].relim()
        xy_min = min(min(self.odom_x), min(self.odom_y))
        xy_max = max(max(self.odom_x), max(self.odom_y))
        self.axs[0, 0].set_xlim(xy_min, xy_max)
        self.axs[0, 0].set_ylim(xy_min, xy_max)

        # Update cmd_vx plot
        self.vx_line.set_data(self.time, self.cmd_vx)
        self.axs[0, 1].relim()
        self.axs[0, 1].autoscale_view()

        # Update cmd_wz plot
        self.wz_line.set_data(self.time, self.cmd_wz)
        self.axs[1, 1].relim()
        self.axs[1, 1].autoscale_view()

        # Refresh plot
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    control_plotter = ControlPlotter()
    
    try:
        rclpy.spin(control_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        control_plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
