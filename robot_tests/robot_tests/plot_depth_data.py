import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

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
        self.line, = self.ax.plot([], [], label='Depth (m)')
        
        self.ax.legend()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Depth (m)')
        self.ax.set_title('Depth Data Over Time')
        
        # Start time for relative time calculation
        self.start_time = self.get_clock().now().nanoseconds / 1E9
        
        # Call the plotting function periodically
        self.timer = self.create_timer(0.1, self.update_plot)

    def listener_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1E9 - self.start_time
        depth = msg.data
        self.depth_map.append(depth)
        self.time.append(current_time)
        
    def update_plot(self):
        # Update plot data
        self.line.set_data(self.time, self.depth_map)
        
        # Update plot limits
        if self.time:
            self.ax.set_xlim(min(self.time), max(self.time))
            self.ax.set_ylim(min(self.depth_map) - 0.1, max(self.depth_map) + 0.1)
        
        # Refresh plot
        plt.draw()
        plt.pause(0.01)

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
