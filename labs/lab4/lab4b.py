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
HEIGHT = int(rc.camera.get_height()*1/7)-1
WIDTH = int(rc.camera.get_width()*2/6)-1
state = 'forward'

########################################################################################
# Functions
########################################################################################

def get_depth_image():
    depth_image = rc.camera.get_depth_image()
    if depth_image is not None:
        depth_image = cv.GaussianBlur(depth_image, (KERNEL_SIZE, KERNEL_SIZE), 0)
        top_left = (int(rc.camera.get_height()*3/7), int(rc.camera.get_width()*2/6))
        bottom_right = (int(rc.camera.get_height()*4/7), int(rc.camera.get_width()*4/6))
        depth_image = rc_utils.crop(depth_image, top_left, bottom_right)
    rc.display.show_depth_image(depth_image)
    
    return depth_image

def get_lidar_scan():
    scan = rc.lidar.get_samples()
    # rc.display.show_lidar(scan)
    
    return scan

def get_wall_distance(scan):
    if scan is None:
        front_distance = None
        right_distance = None
        left_distance = None
    else:
        _, front_distance = rc_utils.get_lidar_closest_point(scan, window=(345, 15))
        _, right_distance = rc_utils.get_lidar_closest_point(scan, window=(0, 180))
        _, left_distance = rc_utils.get_lidar_closest_point(scan, window=(180, 360))
    
    return round(front_distance, 1), round(right_distance, 1), round(left_distance, 1)

def get_depth_image_info(depth_image):
    if depth_image is None:
        front_wall_left_distance = None
        front_wall_right_distance = None
    else:
        front_wall_left_distance = depth_image[int(HEIGHT/2)][0]
        front_wall_right_distance = depth_image[int(HEIGHT/2)][int(WIDTH)]

    return front_wall_left_distance, front_wall_right_distance


def get_speed(front_distance):
    if front_distance is None:
        speed = 0.0
    else:
        speed = rc_utils.remap_range(front_distance, 20, 150, 0.1, 1)
    
    speed = rc_utils.clamp(speed, -1, 1)

    return speed

def get_angle(right_distance, left_distance, front_wall_left_distance, front_wall_right_distance):
    if right_distance is None or front_wall_left_distance is None:
        angle = 0.0
    else:
        distance = right_distance - left_distance
        lidar_angle = rc_utils.remap_range(distance, -36, 36, -1, 1)

        slope = (front_wall_left_distance - front_wall_right_distance) / WIDTH
        wall_angle = 90 - math.degrees(math.atan(slope))
        depth_angle = rc_utils.remap_range(wall_angle, -90, 90, -1, 1)
        print(depth_angle)

        # depth_factor = rc_utils.remap_range(min(front_wall_left_distance, front_wall_right_distance), 20, 100, 0.8, 0.1)
        depth_factor = 0.2
        angle = (lidar_angle * (1 - depth_factor) + depth_angle * depth_factor) / 2

    angle = rc_utils.clamp(angle, -1, 1)

    return angle

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
    depth_image = get_depth_image()
    scan = get_lidar_scan()

    front_distance, right_distance, left_distance = get_wall_distance(scan)
    front_wall_left_distance, front_wall_right_distance = get_depth_image_info(depth_image)

    speed = get_speed(front_distance)
    angle = get_angle(right_distance, left_distance, front_wall_left_distance, front_wall_right_distance)

    rc.drive.set_speed_angle(speed, angle)

    # print(f'front_distance: {front_distance}')
    # print(f'right_distance: {right_distance}')
    # print(f'left_distance: {left_distance}')


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
