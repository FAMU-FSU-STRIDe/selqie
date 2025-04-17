#!/usr/bin/env python3

import time
import rclpy
from cmd import Cmd

from selqie_python.selqie import SELQIE

class SELQIETerminal(Cmd):
    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self):
        super().__init__()
        self._selqie = SELQIE()
        self._selqie.init()
        self._selqie.spin_background()

    def do_exit(self, line : str):
        """ Exit the terminal """
        print("Exiting...")
        self._selqie.stop()
        rclpy.shutdown()
        return True
    
    def do_idle(self, line : str):
        """ Idle the ODrive motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_idle(i)

    def do_ready(self, line : str):
        """ Ready the ODrive motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_ready(i)

    def do_clear_errors(self, line : str):
        """ Clear errors on the ODrive motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_clear_errors(i)

    def do_zero(self, line : str):
        """ Zero the motors """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_position(i, 0.0)

    def do_set_motor_position(self, line : str):
        """ Set the position of a motor """
        args = line.split()
        if len(args) != 2:
            print("Usage: set_motor_position <motor> <position>")
            return
        try:
            self._selqie.set_motor_position(int(args[0]), float(args[1]))
        except ValueError:
            print("Invalid motor or position values")

    def do_set_gains(self, line : str):
        """ Set the gains for the motors """
        args = line.split()
        if len(args) != 3:
            print("Usage: set_gains <p_gain> <v_gain> <vi_gain>")
            return
        try:
            for i in range(self._selqie.NUM_MOTORS):
                self._selqie.set_motor_gains(i, float(args[0]), float(args[1]), float(args[2]))
        except ValueError:
            print("Invalid gain values")

    def do_default(self, line : str):
        """ Set default motor gains and leg positions """
        for i in range(self._selqie.NUM_MOTORS):
            self._selqie.set_motor_gains_default(i)
        for i in range(self._selqie.NUM_LEGS):
            self._selqie.set_leg_position_default(i)

    def do_set_leg_position(self, line : str):
        """ Set the position of a leg """
        args = line.split()
        if len(args) != 4:
            print("Usage: set_leg_position <leg_name/*> <x> <y> <z>")
            return
        try:
            leg = args[0]
            if leg == "*":
                for i in range(self._selqie.NUM_LEGS):
                    self._selqie.set_leg_position(i, float(args[1]), float(args[2]), float(args[3]))
            elif leg in self._selqie.LEG_NAMES:
                self._selqie.set_leg_position(self._selqie.LEG_NAMES.index(leg), float(args[1]), float(args[2]), float(args[3]))
            else:
                print("Invalid leg name")
        except ValueError:
            print("Invalid position values")

    def do_set_leg_force(self, line : str):
        """ Set the force of a leg """
        args = line.split()
        if len(args) != 4:
            print("Usage: set_leg_force <leg_name/*> <x> <y> <z>")
            return
        try:
            leg = args[0]
            if leg == "*":
                for i in range(self._selqie.NUM_LEGS):
                    self._selqie.set_leg_force(i, float(args[1]), float(args[2]), float(args[3]))
            elif leg in self._selqie.LEG_NAMES:
                self._selqie.set_leg_force(self._selqie.LEG_NAMES.index(leg), float(args[1]), float(args[2]), float(args[3]))
            else:
                print("Invalid leg name")
        except ValueError:
            print("Invalid force values")

    def do_print_motor_info(self, line : str):
        """ Print motor info """
        for i in range(self._selqie.NUM_MOTORS):
            print(f"Motor {i}:")
            for attr in ["pos_estimate", "vel_estimate", "torq_estimate"]:
                print(f"  {attr}: {getattr(self._selqie.get_motor_estimate(i), attr)}")
            for attr in ["axis_error", "axis_state", "bus_current", 
                 "bus_voltage", "fet_temperature", "motor_temperature",
                 "iq_measured", "iq_setpoint"]:
                print(f"  {attr}: {getattr(self._selqie.get_motor_info(i), attr)}")

    def do_print_leg_info(self, line : str):
        """ Print leg info """
        for i in range(self._selqie.NUM_LEGS):
            print(f"Leg {self._selqie.LEG_NAMES[i]}:")
            for attr in ["pos_estimate", "vel_estimate", "force_estimate"]:
                vector = getattr(self._selqie.get_leg_estimate(i), attr)
                print(f"  {attr}: x={vector.x}, y={vector.y}, z={vector.z}")

    def do_print_errors(self, line : str):
        """ Print all motor errors """
        iserr = False
        for i in range(self._selqie.NUM_MOTORS):
            if self._selqie.get_motor_info(i).axis_error != 0:
                iserr = True
                print(f"Error on Motor {i}: " + self._selqie.get_motor_error_name(i))
        if not iserr:
            print("No errors on all motors")

    def do_start_recording(self, line : str):
        """ Start rosbag recording of specific topics """
        self._selqie.start_recording()

    def do_stop_recording(self, line : str):
        """ Stop rosbag recording """
        self._selqie.stop_recording()
        
    def do_set_light_brightness(self, line : str):
        """ Set the brightness of the light """
        args = line.split()
        if len(args) != 1:
            print("Usage: set_light_brightness <brightness>")
            return
        try:
            self._selqie.set_vision_lights_brightness(float(args[0]))
        except ValueError:
            print("Invalid brightness value")
            return
        
    def do_set_gait(self, line : str):
        """ Set the gait for the robot """
        args = line.split()
        if len(args) != 1:
            print("Usage: set_gait <gait>")
            return
        try:
            if args[0] == "none":
                self._selqie.set_control_gait('')
            else:
                self._selqie.set_control_gait(args[0])
        except ValueError:
            print("Invalid gait")
            return
        
    def do_cmd_vel(self, line : str):
        """ Publish a Twist message to cmd_vel """
        args = line.split()
        if len(args) != 3:
            print("Usage: cmd_vel <lin_x> <lin_z> <ang_z>")
            return
        try:
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), float(args[2]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_walk(self, line : str):
        """ Walk the robot """
        args = line.split()
        if len(args)!= 2:
            print("Usage: walk <lin_x> <ang_z>")
            return
        try:
            self._selqie.set_control_gait('walk')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), 0.0, float(args[1]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_swim(self, line : str):
        """ Swim the robot """
        args = line.split()
        if len(args) != 2:
            print("Usage: swim <lin_x> <lin_z>")
            return
        try:
            self._selqie.set_control_gait('swim')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), 0.0)
        except ValueError:
            print("Invalid values")
            return
        
    def do_jump(self, line : str):
        """ Jump the robot """
        args = line.split()
        if len(args) != 2:
            print("Usage: jump <lin_x> <lin_z>")
            return
        try:
            self._selqie.set_control_gait('jump')
            time.sleep(0.1)
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]), 0.0)
        except ValueError:
            print("Invalid values")
            return
        
    def do_sink(self, line : str):
        """ Sink the robot """
        self._selqie.set_control_gait('sink')
        time.sleep(0.1)
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)

    def do_stand(self, line : str):
        """ Stand the robot """
        self._selqie.set_control_gait('stand')
        time.sleep(0.1)
        self._selqie.set_control_command_velocity(0.0, 0.0, 0.0)

    def do_set_goal(self, line : str):
        """ Set the goal position for the robot """
        args = line.split()
        if len(args) != 3:
            print("Usage: set_goal <x> <y> <theta>")
            return
        try:
            self._selqie.set_control_goal_pose(float(args[0]), float(args[1]), float(args[2]))
        except ValueError:
            print("Invalid values")
            return
        
    def do_calibrate_imu(self, line : str):
        """ Calibrate the IMU """
        self._selqie.send_localization_calibrate_imu()

    def do_reset_localization(self, line : str):
        """ Reset the localization """
        self._selqie.set_localization_pose_zero()

    def do_reset_map(self, line : str):
        """ Reset the map """
        self._selqie.send_mapping_reset()
        
def main():
    rclpy.init()
    SELQIETerminal().cmdloop()

if __name__ == '__main__':
    main()