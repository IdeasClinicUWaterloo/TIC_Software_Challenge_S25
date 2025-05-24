from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 2

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        control.start_keyboard_input()
        control.start_keyboard_control()
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing


    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall
            print(lidar.checkScan())

    if challengeLevel == 2:
        min_box_size = 8
        flag = True
        while rclpy.ok():
            # Write your solution here for challenge level 2
            (detected, x1, y1, x2, y2) = camera.ML_predict_stop_sign(camera.rosImg_to_cv2())
            box_size = math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
            if (detected == False):
                flag = True
            if(detected == True and box_size > min_box_size and flag == True):
                flag = False
                control.stop_keyboard_control
                control.set_cmd_vel(0,0,3)
                control.start_keyboard_control
            time.sleep(0.1)
    

            
    if challengeLevel == 3:
        tag_instructions = {
            "tag1": (45, 1),
            "tag2": (45, 1),
            "tag3": (45, 1)
        }
        (tag_id, range, bearing, elevation) = camera.estimate_apriltag_pose(camera.rosImg_to_cv2)
        (angle, direction) = tag_instructions.get(tag_id)
        control.rotate(angle, direction)

        



        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
