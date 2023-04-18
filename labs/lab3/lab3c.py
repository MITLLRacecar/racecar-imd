"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

KERNEL_SIZE = 5
DESIRED_DISTANCE = 20

########################################################################################
# Functions
########################################################################################

def get_depth_image(kernel_size):
    depth_image = rc.camera.get_depth_image()
    depth_image = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
    top_left = (int(rc.camera.get_height()*1/3), 0)
    bottom_right = (int(rc.camera.get_height()*2/3), int(rc.camera.get_width()))
    depth_image = rc_utils.crop(depth_image, top_left, bottom_right)

    return depth_image

def get_wall_distance(depth_image):
    if depth_image is None:
        wall_distance = None
    else:
        wall_distance = rc_utils.get_depth_image_center_distance(depth_image)
        wall_distance = round(wall_distance, 1)
    return wall_distance

def get_closest_pixel(depth_image):
    if depth_image is None:
        closest_pixel = None
        min_distance = None
    else:
        closest_pixel = rc_utils.get_closest_pixel(depth_image)
        min_distance = depth_image[closest_pixel[0]][closest_pixel[1]]
        min_distance = round(min_distance, 1)
    return closest_pixel, min_distance

def get_speed(wall_distance, min_distance):
    if wall_distance is None:
        speed = 0.0
    else:
    # if(min_distance < DESIRED_DISTANCE+55 and abs(wall_distance - min_distance) > 13):
        speed = rc_utils.remap_range(min_distance, 0, DESIRED_DISTANCE+55, -1, 0)
        print("min_distance_speed")
    # else:
        if(wall_distance >= DESIRED_DISTANCE):
            speed = rc_utils.remap_range(wall_distance, DESIRED_DISTANCE, 150, 0, 1)
        else:
            speed = rc_utils.remap_range(wall_distance, 0, DESIRED_DISTANCE, -1, 0)
        print("wall_distance_speed")

        speed = rc_utils.clamp(speed, -1, 1)
        if(abs(speed) < 0.009):
            speed = 0.0
    return speed

def get_angle(closest_pixel, wall_distance, min_distance):
    if closest_pixel is None:
        angle = 0.0
    else:
        current_angle = closest_pixel[1]
        angle = rc_utils.remap_range(current_angle, 0, rc.camera.get_width(), -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)
        # if(wall_distance < DESIRED_DISTANCE or (min_distance < DESIRED_DISTANCE+55 and abs(wall_distance - min_distance) > 13)):
        #     angle = -angle
        if(abs(angle) < 0.25):
            angle = 0.0
    return angle

def correct_speed_angle(speed, angle, wall_distance, min_distance):
    if(angle != 0 and abs(angle) > 0.8):
        speed = -1
        angle = -angle
    return speed, angle

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 20 cm away from the closest wall with the car directly facing
    # the wall
    depth_image = get_depth_image(KERNEL_SIZE)
    wall_distance = get_wall_distance(depth_image)
    closest_pixel, min_distance = get_closest_pixel(depth_image)

    speed = get_speed(wall_distance, min_distance)
    angle = get_angle(closest_pixel, wall_distance, min_distance)

    speed, angle = correct_speed_angle(speed, angle, wall_distance, min_distance)
    

    print(f'wall_distance: {wall_distance}')
    print(f'min_distance: {min_distance}')
    print(f'speed: {speed}')
    print(f'angle: {angle}')
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
