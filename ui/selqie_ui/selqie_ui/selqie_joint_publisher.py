#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from actuation_msgs.msg import MotorEstimate

def QOS_FAST() -> QoSProfile:
    """Get a QoSProfile with best-effort reliability and a depth of 10."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

class SELQIEJointPublisher(Node):
    def __init__(self):
        super().__init__('selqie_joint_publisher')
        
        # Subscriber to motor estimates
        self.NUM_MOTORS = 8
        self._motor_estimates = [MotorEstimate() for _ in range(self.NUM_MOTORS)]
        self._motor_estimate_subscribers = []
        for i in range(self.NUM_MOTORS):
            motor_estimate_callback = lambda msg, i=i: self._motor_estimates.__setitem__(i, msg)
            self._motor_estimate_subscribers.append(self.create_subscription(MotorEstimate, f'odrive{i}/estimate', motor_estimate_callback, QOS_FAST()))
            
        # Five bar linkage parameters
        self.LEG_LINK_1_LENGTH = 0.066
        self.LEG_LINK_2_LENGTH = 0.15
        self.Y_AXIS_FLIP = [-1, -1, -1, -1, 1, 1, 1, 1]
            
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
            
        # Timer for publishing joint states
        self.PUBLISH_FREQUENCY = 20.0
        self.create_timer(1.0/self.PUBLISH_FREQUENCY, self.timer_callback)
            
    def timer_callback(self):
        """Timer callback to publish joint states."""
        
        # Create a JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(0, self.NUM_MOTORS, 2):
            # Calculate the angles for the five bar linkage
            thetaA = self._motor_estimates[i].pos_estimate
            thetaB = self._motor_estimates[i+1].pos_estimate
            alpha = 0.5 * (math.pi - self.Y_AXIS_FLIP[i] * thetaA - self.Y_AXIS_FLIP[i+1] * thetaB)
            gamma = math.asin(self.LEG_LINK_1_LENGTH * math.sin(alpha) / self.LEG_LINK_2_LENGTH)
            beta = alpha + gamma
            
            # Append the joint names and corresponding angles to the JointState message
            joint_state.name.append(f'motor{i}_link')
            joint_state.position.append(thetaA)
            joint_state.name.append(f'knee{i}_link')
            joint_state.position.append(beta)
            joint_state.name.append(f'motor{i+1}_link')
            joint_state.position.append(thetaB)
            joint_state.name.append(f'knee{i+1}_link')
            joint_state.position.append(beta)
            
        # Publish the joint state message
        self.joint_state_publisher.publish(joint_state)
            
# Entry point for the node
def main(args=None):
    rclpy.init(args=args)
    node = SELQIEJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()