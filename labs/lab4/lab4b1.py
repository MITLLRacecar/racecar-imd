"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Follow the wall to the right of the car without hitting anything.
    pass
    scan = rc.lidar.get_samples()

    _, left_point = rc_utils.get_lidar_closest_point(scan, (-48, -42))
    _, right_point = rc_utils.get_lidar_closest_point(scan, (42, 48))
    print (left_point, right_point)
    speed = 1
    midpoint = right_point - left_point
    kp = 0.02
    angle = kp * midpoint
    angle = rc_utils.clamp(angle, -1, 1)
    if -0.2 < angle < 0.2:
        angle = 0
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
