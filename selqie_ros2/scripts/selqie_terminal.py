#!/usr/bin/env python3

import os
import time
import rclpy
from cmd import Cmd

from selqie_ros2.selqie import SELQIE

def PRINT_MOTOR_INFO(motor_info):
    for attr in ["axis_error", "axis_state", "bus_current", 
                 "bus_voltage", "fet_temperature", "motor_temperature",
                 "iq_measured", "iq_setpoint"]:
        print(f"  {attr}: {getattr(motor_info, attr)}")

def PRINT_MOTOR_ESTIMATE(motor_estimate):
    for attr in ["pos_estimate", "vel_estimate", "torq_estimate"]:
        print(f"  {attr}: {getattr(motor_estimate, attr)}")

def PRINT_LEG_INFO(leg_estimate):
    for attr in ["pos_estimate", "vel_estimate", "force_estimate"]:
        vector = getattr(leg_estimate, attr)
        print(f"  {attr}: x={vector.x}, y={vector.y}, z={vector.z}")

class SELQIETerminal(Cmd):
    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self):
        super().__init__()
        self._selqie = SELQIE()
        self._selqie.spin()

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
    
    def do_run_trajectory(self, line : str):
        """ Run a trajectory file or sequence of files """
        args = line.split()
        if len(args) % 3 != 0:
            print("Usage: run_trajectory <file1> <num_loops1> <frequency1> <file2> <num_loops2> <frequency2> ...")
            return
        try:
            for i in range(0, len(args), 3):
                file = args[0]
                num_loops = int(args[i+1])
                frequency = float(args[i+2])
                trajectories = self._selqie.get_leg_trajectories_from_file(file, frequency)
                print(f"Running trajectory for {num_loops} loops at {frequency} Hz")
                for i in range(num_loops):
                    print(f"  Loop {i+1}/{num_loops}")
                    self._selqie.run_leg_trajectories(trajectories)
                    time.sleep(1.0 / frequency)
                print("Finished trajectory")
        except ValueError:
            print("Invalid number of loops or frequency")
        except FileNotFoundError:
            print("File not found")
            
    def complete_run_trajectory(self, text, line, begidx, endidx):
        """ Autocomplete for run_trajectory """
        if len(line.split()) % 3 == 1 or len(line.split()) % 3 == 2:
            files = os.listdir(self._selqie.TRAJECTORIES_FOLDER)
            return [f for f in files if f.startswith(text)]
        return []

    def do_print_motor_info(self, line : str):
        """ Print motor info """
        for i in range(self._selqie.NUM_MOTORS):
            print(f"Motor {i}:")
            PRINT_MOTOR_ESTIMATE(self._selqie.get_motor_estimate(i))

    def do_print_leg_info(self, line : str):
        """ Print leg info """
        for i in range(self._selqie.NUM_LEGS):
            print(f"Leg {self._selqie.LEG_NAMES[i]}:")
            PRINT_LEG_INFO(self._selqie.get_leg_estimate(i))

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
        if len(args) != 2:
            print("Usage: cmd_vel <lin_x> <ang_z>")
            return
        try:
            self._selqie.set_control_command_velocity(float(args[0]), float(args[1]))
        except ValueError:
            print("Invalid values")
            return

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
        
def main():
    rclpy.init()
    SELQIETerminal().cmdloop()

if __name__ == '__main__':
    main()