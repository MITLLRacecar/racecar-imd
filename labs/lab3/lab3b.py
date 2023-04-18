"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
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

# >> Constants
ORANGE = ((10, 100, 100), (20, 255, 255))
MIN_CONTOUR_AREA = 30
KERNEL_SIZE = 5
DESIRED_DISTANCE = 30

########################################################################################
# Functions
########################################################################################

def get_color_image():
    image = rc.camera.get_color_image()
    return image

def get_depth_image(kernel_size):
    depth_image = rc.camera.get_depth_image()
    depth_image = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
    return depth_image

def get_contour_center(image, color):

    if image is None:
        contour_center = None
    else:
        # Find largest contour
        contours = rc_utils.find_contours(image, color[0], color[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is None:
            contour_center = None
        else:
            # Find contour center
            contour_center = rc_utils.get_contour_center(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

            # Display color image
            rc.display.show_color_image(image)

    return contour_center
        
def get_cone_distance(depth_image, contour_center):
    if contour_center is None:
        cone_distance = None
    else:
        if depth_image is None:
            cone_distance = None
        else:
            cone_distance = depth_image[contour_center[0]][contour_center[1]]
            cone_distance = round(cone_distance, 1)
    print(f'cone_distance: {cone_distance}')
    return cone_distance
        
def get_speed(contour_center, cone_distance):
    if contour_center is None:
        speed = 0
    else:
        if(cone_distance >= DESIRED_DISTANCE):
            speed = rc_utils.remap_range(cone_distance, DESIRED_DISTANCE, 150, 0, 1)
        else:
            speed = rc_utils.remap_range(cone_distance, 0, DESIRED_DISTANCE, -1, 0)

        speed = rc_utils.clamp(speed, -1, 1)
        if(abs(speed) < 0.0035):
            speed = 0.0
    print(f'speed: {speed}')
    return speed

def get_angle(contour_center, cone_distance):
    if contour_center is None:
        angle = 0.0
    else:
        current_angle = contour_center[1]
        angle = rc_utils.remap_range(current_angle, 0, rc.camera.get_width(), -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)
        # Drive in reverse if too close to cone
        if(cone_distance < DESIRED_DISTANCE):
            angle = -angle
    print(f'angle: {angle}')
    return angle

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3B - Depth Camera Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 30 cm away from the closest orange cone.
    # Use both color and depth information to handle cones of multiple sizes.
    # You may wish to copy some of your code from lab2b.py
    image = get_color_image()
    depth_image = get_depth_image(KERNEL_SIZE)

    contour_center = get_contour_center(image, ORANGE)
    cone_distance = get_cone_distance(depth_image, contour_center)

    speed = get_speed(contour_center, cone_distance)
    angle = get_angle(contour_center, cone_distance)

    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
