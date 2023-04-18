"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
SPEED = 0
DISTANCE = 0
global EMERGENCY_STOP
RED = ((0, 50, 50), (10, 255, 255))
ORANGE = ((10, 100, 100), (20, 255, 255))

EMERGENCY_STOP = False

########################################################################################
# Functions
########################################################################################

def getSpeed(depth_image):
    delta_distance = rc_utils.get_depth_image_center_distance(depth_image) - DISTANCE
    delta_time = rc.get_delta_time()

    # print(f'distance: {delta_time}')
    # print(f'time: {delta_time}')
    # print(f'speed: {delta_distance/delta_time}')

    return delta_distance/delta_time

def check_if_ramp(depth_image):
    mid = int(rc.camera.get_width()*1/6)
    
    start = int(rc.camera.get_height()*7/15*2/3)
    end = int(rc.camera.get_height()*7/15*4/5)

    distance = depth_image[start][mid]
    delta_distance = distance - depth_image[start+1][mid]
    distance = depth_image[start+1][mid]

    for i in range(start, end):
        if(delta_distance == distance - depth_image[i][mid]):
            print("not ramp")
            return False

    print("SLOPE")
    return True

def get_mask(
    image: NDArray[(Any, Any, 3), np.uint8],
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int]
) -> NDArray[Any, Any]:
    """   
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.
    
    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)
    
    # TODO: Use the cv.cvtColor function to switch our BGR colors to HSV colors
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # TODO: Use the cv.inRange function to highlight areas in the correct range
    mask = cv.inRange(image, hsv_lower, hsv_upper)
    
    return mask


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    depth_image = rc.camera.get_depth_image()
    DISTANCE = rc_utils.get_depth_image_center_distance(depth_image)

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Right bumper = override safety stop\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = print current speed and angle\n"
        "    B button = print the distance at the center of the depth image"
    )

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    depth_image = rc.camera.get_depth_image()
    center_distance = rc_utils.get_depth_image_center_distance(depth_image)

    image = rc.camera.get_color_image()
    contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    contour = rc_utils.get_largest_contour(contours, 10)

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    # Allow the user to override safety stop by holding the right bumper.

    SPEED = getSpeed(depth_image)

    # if contour is None:
    top = (int(rc.camera.get_height()*1/5), int(rc.camera.get_width()*1/3))
    bottom = (int(rc.camera.get_height()*2/3), int(rc.camera.get_width()*2/3))

    depth_image = rc_utils.crop(depth_image, top, bottom)

    closest_pixel = rc_utils.get_closest_pixel(depth_image)

    # if(EMERGENCY_STOP):
    #     speed = 0

    if(depth_image[closest_pixel[0], closest_pixel[1]] < 80):
        if(not check_if_ramp(depth_image)):
            EMERGENCY_STOP = True
            if(depth_image[closest_pixel[0], closest_pixel[1]] < 20):
                speed = -1
            else:
                speed = 0
    # Display the current depth image
    rc.display.show_depth_image(depth_image)
    # else:
    #     mask = get_mask(image, RED[0], RED[1])
    #     masked = cv.bitwise_and(depth_image, depth_image, mask=mask)

    #     # Display the current depth image
    #     rc.display.show_depth_image(masked)

    #     closest_pixel = rc_utils.get_closest_pixel(depth_image)

    #     if(depth_image[closest_pixel[0], closest_pixel[1]] < 100):
    #         EMERGENCY_STOP = True
    #         if(depth_image[closest_pixel[0], closest_pixel[1]] < 20):
    #             speed = -1
    #         else:
    #             speed = 0

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
