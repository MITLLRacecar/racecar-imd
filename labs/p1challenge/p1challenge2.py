"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
Phase 1 Challenge - Cone Slaloming
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import math as mt
sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

class State(IntEnum):
    search  = 0
    approach_red = 1
    approach_blue = 2
    turn_red = 3
    turn_blue = 4
    stop = 5
    

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

curr_state: State = State.search

speed = 0.0
angle = 0.0
counter = 0.0

#HSV Values
BLUE = ((100, 150, 150), (130, 255, 255))  # The HSV range for the color blue
RED  = ((165, 0, 0),(179, 255, 255)) 
##Contour stuff
MIN_CONTOUR_AREA = 800

#Distance threshold before turning
DIST = 80
# Cone recovery
RECOVER_BLUE = False
RECOVER_RED = False
#Speed constants
APPROACH_SPEED = 1
TURN_SPEED = 0.65
#Angle constants
RECOVER_ANGLE = 0.95
TURN_ANGLE =  1
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
    print(">> Phase 1 Challenge: Cone Slaloming")

def get_contour(HSV, MIN_CONTOUR_AREA = 30):
    image = rc.camera.get_color_image()
    if image is None:
        return None
    else:
        contours = rc_utils.find_contours(image, HSV[0], HSV[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        return contour

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.
    global angle, speed, RECOVER_BLUE, RECOVER_RED
    global curr_state, counter
    global DIST, RED, BLUE, MIN_CONTOUR_AREA

    image = rc.camera.get_color_image()
    depth_image_original = (rc.camera.get_depth_image() - 0.01) % 10000
    depth_image_original= cv.GaussianBlur(depth_image_original,(3,3),0)
    
    red_contour = get_contour(RED, MIN_CONTOUR_AREA)
    blue_contour = get_contour(BLUE, MIN_CONTOUR_AREA)
    
    red_center = rc_utils.get_contour_center(red_contour) if red_contour is not None else None
    blue_center = rc_utils.get_contour_center(blue_contour) if blue_contour is not None else None
    
    red_depth = depth_image_original[red_center[0]][red_center[1]] if red_contour is not None else 0.0
    blue_depth = depth_image_original[blue_center[0]][blue_center[1]] if blue_contour is not None else 0.0
    
    # Identification of the closest object
    
    if curr_state == State.search:
        if RECOVER_RED: 
            angle = -RECOVER_ANGLE
        if RECOVER_BLUE:
            angle = RECOVER_ANGLE
        if red_depth < blue_depth and red_depth!=0:
            curr_state = State.approach_red
        if blue_depth < red_depth and blue_depth!=0:
            curr_state  = State.approach_blue
        elif red_depth != 0:
            curr_state = State.approach_red
        elif blue_depth !=0:
            curr_state = State.approach_blue

    if curr_state == State.approach_red:
        if RECOVER_BLUE: RECOVER_BLUE = False
        if red_depth == 0.0: 
            curr_state = State.search
        elif red_depth < DIST: 
            curr_state = State.turn_red  
        else:
            rc_utils.draw_circle(image,red_center)
            angle = rc_utils.remap_range(red_center[1], 0, rc.camera.get_width(), -1,1, True) 
    
    if curr_state == State.approach_blue:
        if RECOVER_RED: RECOVER_RED = False
        if blue_depth == 0.0: 
            curr_state = State.search
        elif blue_depth < DIST: 
            curr_state = State.turn_blue
        else:
            rc_utils.draw_circle(image, blue_center)
            angle = rc_utils.remap_range(blue_center[1], 0, rc.camera.get_width(), -1,1,True) 
    
    if curr_state == State.turn_red:
        counter += rc.get_delta_time()
        if counter < 0.85:
            angle = TURN_ANGLE
        elif counter < 1: 
            angle = 0
        else:
            counter = 0
            RECOVER_RED = True
            curr_state = State.search

    if curr_state == State.turn_blue:
        counter += rc.get_delta_time()
        if counter < 0.85:
            angle = -TURN_ANGLE
        elif counter < 1: 
            angle = 0
        else:
            counter = 0
            RECOVER_BLUE = True
            curr_state = State.search
    
    if curr_state == (State.approach_blue or State.approach_red or State.search):
        speed = APPROACH_SPEED
    else:
        speed = TURN_SPEED
    
    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_color_image(image)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()