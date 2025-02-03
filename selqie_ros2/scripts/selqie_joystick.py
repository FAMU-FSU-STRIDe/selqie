#!/usr/bin/env python3

import math

import rclpy
from sensor_msgs.msg import Joy
from robot_msgs.msg import ODriveConfig

from selqie_ros2.selqie import SELQIE

STANDING_LEG_POSITION = [0.0, 0.0, -0.18914]
LANDING_LEG_POSITION = [0.0, 0.0, -0.18914]

STANDING_GAINS = [50.0, 0.15, 0.30]
WALKING_GAINS = [50.0, 0.15, 0.30]
JUMPING_GAINS = [150.0, 0.05, 0.10]
SWIMMING_GAINS = [150.0, 0.05, 0.10]
LANDING_GAINS = [10.0, 0.10, 0.20]

class SELQIEJoystick():
    def __init__(self):
        self._selqie = SELQIE()

        self.last_msg = None
        self.joy_sub = self._selqie.create_subscription(
            Joy, 'joy', self._joy_callback, 10)
        
        self._selqie.get_logger().info("SELQIE Joystick Node Started")
        
        try:
            self._selqie.spin()
        except KeyboardInterrupt:
            self._selqie.destroy_node()
        
    def _joy_callback(self, msg : Joy):
        if self.last_msg is None:
            pass
        elif msg.buttons[0] == 1 and self.last_msg.buttons[0] == 0:
            # 1 : Walk
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *WALKING_GAINS)
            self._selqie.set_control_gait('walk')
            self._selqie.get_logger().info('Walking...')
        elif msg.buttons[1] == 1 and self.last_msg.buttons[1] == 0:
            # 2 : Jump
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *JUMPING_GAINS)
            self._selqie.set_control_gait('jump')
            self._selqie.get_logger().info('Jumping...')
        elif msg.buttons[2] == 1 and self.last_msg.buttons[2] == 0:
            # 3 : Swim
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *SWIMMING_GAINS)
            self._selqie.set_control_gait('swim')
            self._selqie.get_logger().info('Swimming...')
        elif msg.buttons[3] == 1 and self.last_msg.buttons[3] == 0:
            # 4 : Crawl
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *WALKING_GAINS)
            self._selqie.set_control_gait('crawl')
            self._selqie.get_logger().info('Crawling...')
        elif msg.buttons[4] == 1 and self.last_msg.buttons[4] == 0:
            # LB : Stand
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *STANDING_GAINS)
            for l in range(self._selqie.NUM_LEGS):
                self._selqie.set_leg_position_default(l)
            self._selqie.get_logger().info('Standing...')
        elif msg.buttons[5] == 1 and self.last_msg.buttons[5] == 0:
            # RB : Land
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *LANDING_GAINS)
            for l in range(self._selqie.NUM_LEGS):
                self._selqie.set_leg_position_default(l)
            self._selqie.get_logger().info('Landing...')
        elif msg.buttons[6] == 1 and self.last_msg.buttons[6] == 0:
            # LT
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(m, *STANDING_GAINS)
                self._selqie.set_motor_position(m, 0.0)
            self._selqie.get_logger().info('Zeroed ODrives')
        elif msg.buttons[7] == 1 and self.last_msg.buttons[7] == 0:
            # RT
            pass
        elif msg.buttons[8] == 1 and self.last_msg.buttons[8] == 0:
            # 9 : Ready
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_state(m, ODriveConfig.AXIS_STATE_CLOSED_LOOP_CONTROL)
            self._selqie.get_logger().info('Readied ODrives')
        elif msg.buttons[9] == 1 and self.last_msg.buttons[9] == 0:
            # 10 : Idle
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_clear_errors(m)
            self._selqie.get_logger().info('Cleared ODrive errors')
            for m in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_state(m, ODriveConfig.AXIS_STATE_IDLE)
            self._selqie.get_logger().info('Idled ODrives')
        elif msg.axes[4] >= 0.5 and self.last_msg.axes[4] < 0.5:
            # Cross left
            pass
        elif msg.axes[4] <= -0.5 and self.last_msg.axes[4] > -0.5:
            # Cross right
            pass
        elif msg.axes[5] >= 0.5 and self.last_msg.axes[5] < 0.5:
            # Cross up
            pass
        elif msg.axes[5] <= -0.5 and self.last_msg.axes[5] > -0.5:
            # Cross down
            pass

        la_x = int(msg.axes[1] * 5) / 5.0 # left stick x
        la_y = int(msg.axes[0] * 5) / 5.0 # left stick y
        ra_x = int (msg.axes[3] * 5) / 5.0 # right stick x
        
        if la_x > 0.0:
            n_vx = math.sqrt(la_x)
        else:
            n_vx = -math.sqrt(-la_x)
        if la_y > 0.0:
            n_wz = math.sqrt(la_y)
        else:
            n_wz = -math.sqrt(-la_y)
            
        ALPHA = 0.25
        cmd_vx = n_vx / (abs(n_wz) + 1) * ALPHA * max(abs(n_vx), abs(n_wz))
        cmd_wz = n_wz / (abs(n_vx) + 1) * ALPHA * max(abs(n_vx), abs(n_wz))
        cmd_vz = ra_x * ALPHA
        
        self._selqie.set_control_command_velocity(cmd_vx, cmd_wz, cmd_vz)
            
        self.last_msg = msg


def main(args=None):
    rclpy.init(args=args)
    SELQIEJoystick()

if __name__ == '__main__':
    main()