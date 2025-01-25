#!/usr/bin/env python3

import rclpy

from selqie_ros2.selqie import SELQIE

class SELQIEJoystick():
    def __init__(self):
        self._selqie = SELQIE()


def main(args=None):
    rclpy.init(args=args)
    selqie_joystick = SELQIEJoystick()
    rclpy.shutdown()

if __name__ == '__main__':
    main()