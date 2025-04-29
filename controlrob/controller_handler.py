#!/usr/bin/env python3

# ROS node to command an Endpoint to a HiWonder xArm 1S using Nintendo Switch controller. 
# Peter Adamczyk, University of Wisconsin - Madison --- modified by Ethan Bjerke to take controller input
# Updated 2024-11-14
 
import pygame
import time

import rclpy
from rclpy.node import Node
import tkinter as tk
import threading
import traceback 
import numpy as np

from std_msgs.msg import Float64
from xarmrob_interfaces.msg import ME439PointXYZ

coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'

class ControllerHandler(Node): 
    def __init__(self): 
        super().__init__('controller_handler')
        
        self.xyz_goal = [0.165, 0.0, 0.155] # roughly upright neutral with wrist at 45 degrees. Formally: [0.1646718829870224, 0.0, 0.1546700894832611]
        self.speed = 0.001 # in m/(some arbitrary amount of time)
        self.gripper_angle = 0.0
        self.grip_close_speed = 0.005
        
        self.pub_gripper_angle = self.create_publisher(Float64, '/gripper_angle_desired', 1)

        # =============================================================================
        #   # Publisher for the Endpoint goal. 
        # =============================================================================
        self.pub_endpoint_desired = self.create_publisher(ME439PointXYZ,'/endpoint_desired',1)
        # Create the message, with a nominal pose
        self.endpoint_desired_msg = ME439PointXYZ()
        self.endpoint_desired_msg.xyz = self.xyz_goal 

        # command frequency parameter: how often we expect it to be updated    
        self.command_frequency = self.declare_parameter('command_frequency',10).value
        self.movement_time_ms = round(1000/self.command_frequency)  # milliseconds for movement time. 
        
        # Set up a timer to send the commands at the specified rate. 
        self.timer = self.create_timer(self.movement_time_ms/1000, self.send_data)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No controllers found. Please connect the Nintendo Switch controller")
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    # Callback to publish the endpoint at the specified rate. 
    def send_data(self):
        self.endpoint_desired_msg.xyz = self.xyz_goal 
        self.pub_endpoint_desired.publish(self.endpoint_desired_msg)

        angle_msg = Float64()
        angle_msg.data = self.gripper_angle
        self.pub_gripper_angle.publish(angle_msg)

    #%% Displace the end of the arm      
    def displace_x(self, dx):
        self.xyz_goal[0] = self.xyz_goal[0] + float(dx)

    def displace_y(self, dy):
        self.xyz_goal[1] = self.xyz_goal[1] + float(dy)
        
    def displace_z(self, dz):
        self.xyz_goal[2] = self.xyz_goal[2] + float(dz)

    def change_grip_angle(self, delta):
        self.gripper_angle = self.gripper_angle + delta
        
    #%% Poll Nintendo Switch Controller for input changes, than change the xyz_goal
    def poll_controller_input(self): 
        self.prev_axes = [0.0] * self.joystick.get_numaxes()

        try:
            while True:
                pygame.event.pump()
                
                for i in range(self.joystick.get_numbuttons()):
                    if self.joystick.get_button(i):
                        if i == 4:
                            self.displace_z(-1.0 * self.speed)
                            # LT to move the arm down on the Z-Axis
                        elif i == 5:
                            self.displace_z(self.speed)
                            # RT to move the arm up on the Z-Axis

                        if i == 6:
                            self.change_grip_angle(-1.0 * self.grip_close_speed)
                        elif i == 7:
                            self.change_grip_angle(1.0 * self.grip_close_speed)

                        print(f"Button {i} pressed")

                for i in range(self.joystick.get_numaxes()):
                    axis_value = self.joystick.get_axis(i)
                    if abs(axis_value - self.prev_axes[i]) > 0.01:
                        axis_name = "Unknown"
                        if i == 0: 
                            axis_name = "Left Stick X"
                        elif i == 1:
                            axis_name = "Left Stick Y"
                        elif i == 2: 
                            axis_name = "Right Stick X"
                        elif i == 3: 
                            axis_name = "Right Stick Y"
                        elif i == 4: 
                            axis_name = "Left Trigger"
                        elif i == 5: 
                            axis_name = "Right Trigger"
                         
                        print(f"Axis {i} ({axis_name}): {axis_value:.2f}")
                        self.prev_axes[i] = axis_value

                    if abs(axis_value) < 0.01: axis_value = 0.0
                    if i == 0:
                        self.displace_x(axis_value * self.speed)
                    elif i == 1:
                        self.displace_y(axis_value * self.speed)

                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nTerminated controller polling...")
        finally:
            pygame.quit()

def main(args=None):
    try: 
        rclpy.init(args=args)
        controller_handler_instance = ControllerHandler()  
        
        # Run the controller events poll loop in a separate thread in order to isolate from ROS loop. 
        thrd = threading.Thread(target=controller_handler_instance.poll_controller_input)
        thrd.start()
        
        # "Spin" the node so that the timer callback will execute. 
        rclpy.spin(controller_handler_instance)
        
    except: 
        traceback.print_exc()
        


if __name__ == '__main__':
    main()