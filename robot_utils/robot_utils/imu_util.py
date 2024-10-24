import rclpy
from rclpy.node import Node
from cmd import Cmd
import threading
from sensor_msgs.msg import Imu
from robot_utils.utils.ros_util_functions import *

class ImuUtilityNode(Node):
    def __init__(self):
        super().__init__('imu_terminal_node')
        
        self.imu_data : Imu = None
        imu_callback = lambda msg : setattr(self, 'imu_data', msg)
        self.sub = self.create_subscription(Imu, '/imu/data', imu_callback, qos_reliable())
        

class ImuTerminal(Cmd):
    intro = 'Welcome to the IMU terminal. Type help or ? to list commands.\n'
    prompt = 'IMU> '
    
    def __init__(self, node_instance : ImuUtilityNode):
        super().__init__()
        self.node = node_instance

    def do_exit(self, line):
        """
        Exit the terminal.
        """
        print("Exiting...")
        rclpy.shutdown()
        return True
    
    def do_rpy(self, line):
        """
        Get the roll, pitch, and yaw values of the IMU.
        """
        if self.node.imu_data is not None:
            roll, pitch, yaw = quaternion_to_euler(self.node.imu_data.orientation)
            print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        else:
            print("IMU data not available.")
            
    def do_calibrate(self, line):
        """
        Get the current gravity of the IMU.
        """
        args = line.split()
        duration = 10.0
        if len(args) > 0:
            duration = float(args[0])
        
        if self.node.imu_data is not None:
            print(f"Calibrating for {duration} seconds...")
            start = time.time()
            accel = [0, 0, 0]
            count = 0
            while time.time() - start < duration:
                accel[0] += self.node.imu_data.linear_acceleration.x
                accel[1] += self.node.imu_data.linear_acceleration.y
                accel[2] += self.node.imu_data.linear_acceleration.z
                count += 1
                time.sleep(0.1)
            accel[0] /= count
            accel[1] /= count
            accel[2] /= count
            mag = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
            print(f"Gravity vector: {accel}")
            print(f"Gravity magnitude: {mag}")
        else:
            print("IMU data not available.")

def main():
    rclpy.init()
    node = ImuUtilityNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    ImuTerminal(node).cmdloop()
    spin_thread.join()
    node.destroy_node()

if __name__ == '__main__':
    main()
