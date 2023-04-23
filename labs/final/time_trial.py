"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Time Trial
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import Enum, IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

class State:
    mode = 'slalom'
    color = None
    turn = None

class Orientation(Enum):
    UP = 0
    LEFT = 1
    DOWN = 2
    RIGHT = 3

rc = racecar_core.create_racecar()

# Add any global variables here
KERNEL_SIZE = (5, 5)
HEIGHT = rc.camera.get_height()
WIDTH = rc.camera.get_width()
CROP_FLOOR = ((360, 0), (HEIGHT, WIDTH))
CROP_RIGHT = ((0, int(WIDTH/2)), (HEIGHT, WIDTH))
CROP_LEFT = ((0, 0), (HEIGHT, int(WIDTH/2)))
MIN_CONTOUR_AREA = 1
state = State()
image = None
depth_image = None
scan = None
speed = 0.0
angle = 0.0
contour_center = None
contour = None
max_depth = 400

colors = {
    'ORANGE': ((10, 50, 50), (20, 255, 255)),
    'BLUE': ((100, 150, 150), (130, 255, 255)),
    'GREEN': ((40, 50, 50), (80, 255, 255)),
    'RED': ((170, 50, 50), (10, 255, 255)),
    'PURPLE': ((110,59,50), (165,255,255))
}
potential_colors = [
    ((10, 50, 50), (20, 255, 255),'ORANGE'),
    ((100, 150, 50), (110, 255, 255),'BLUE'),
    ((40, 50, 50), (80, 255, 255),'GREEN'),  # The HSV range for the color green
    ((170, 50, 50), (10, 255, 255),'RED'),
    ((110,59,50), (165,255,255),'PURPLE')
]

########################################################################################
# Functions
########################################################################################

def collect_data():
    global image, depth_image, scan
    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    depth_image = (depth_image - 0.01) % 10000
    depth_image = cv.GaussianBlur(depth_image, KERNEL_SIZE, 0)
    scan = rc.lidar.get_samples()

def set_state():
    global image, depth_image
    if image is not None and depth_image is not None:
        markers = rc_utils.get_ar_markers(image, potential_colors)
        # image = rc_utils.draw_ar_markers(image, markers)
        closest_marker = (None, float('inf'))
        for marker in markers:
            corners = marker.get_corners()
            top_left = corners[0]
            top_right = corners[1]
            bottom_right = corners[2]
            bottom_left = corners[3]
            min_distance = max(depth_image[top_left[0]][top_left[1]], depth_image[top_right[0]][top_right[1]], depth_image[bottom_right[0]][bottom_right[1]], depth_image[bottom_left[0]][bottom_left[1]])
            if min_distance < closest_marker[1] and min_distance < 200:
                closest_marker = (marker, min_distance)

            marker = closest_marker[0]
            if marker is not None:
                id = marker.get_id()
                color = marker.get_color()
                orientation = marker.get_orientation()
                corners = marker.get_corners()

                if color == 'PURPLE' or color == 'ORANGE':
                    state.mode = 'lane'
                    state.color = color
                elif id == 0:
                    state.turn = -1
                    # if state.mode == 'line':
                    #     state.color = color
                elif id == 1:
                    state.turn = 1
                    # if state.mode == 'line':
                    #     state.color = color
                elif id == 199:
                    if orientation.value == Orientation.LEFT.value:
                        state.turn = -1
                    elif orientation.value == Orientation.RIGHT.value:
                        state.turn = 1
                    # if state.mode == 'line':
                    #     state.color = color
                elif id == 2:
                    if color == 'not detected':
                        state.mode = 'slalom'
                        state.color = None
                        state.turn  = None
                    else:
                        state.mode = 'line'
                        state.color = color
                elif id == 3:
                    state.mode = 'wall'
                    state.color = None

def follow_line():
    global image, depth_image, contour_center, contour, speed, angle
    image_copy = image

    image_copy = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
    depth_image_copy = rc_utils.crop(depth_image, CROP_FLOOR[0], CROP_FLOOR[1])

    if state.color is not None:
        contours = rc_utils.find_contours(image_copy, colors[state.color][0], colors[state.color][1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
        
    if (state.color is not None and contour is not None and depth_image_copy[contour_center[0]][contour_center[1]] > 150) or state.color is None:
        state.color = None

        contours_red = rc_utils.find_contours(image_copy, colors['RED'][0], colors['RED'][1])
        contours_green = rc_utils.find_contours(image_copy, colors['GREEN'][0], colors['GREEN'][1])
        contours_blue = rc_utils.find_contours(image_copy, colors['BLUE'][0], colors['BLUE'][1])

        contour_red = rc_utils.get_largest_contour(contours_red, MIN_CONTOUR_AREA)
        contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)
        contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA)

        # contours = [contour_red, contour_green, contour_blue]

        # contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour_red is not None:
            contour_area_red = rc_utils.get_contour_area(contour_red)
        else:
            contour_area_red = 0
        if contour_green is not None:
            contour_area_green = rc_utils.get_contour_area(contour_green)
        else:
            contour_area_green = 0
        if contour_blue is not None:
            contour_area_blue = rc_utils.get_contour_area(contour_blue)
        else:
            contour_area_blue = 0

        if contour_red is not None and contour_area_red > contour_area_green and contour_area_red > contour_area_blue:
            contour = contour_red
        elif contour_green is not None and contour_area_green > contour_area_red and contour_area_green > contour_area_blue:
            contour = contour_green
        elif contour_blue is not None and contour_area_blue > contour_area_red and contour_area_blue > contour_area_green:
            contour = contour_blue

        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)

    if contour is not None:
        rc_utils.draw_contour(image_copy, contour)
        rc_utils.draw_circle(image_copy, contour_center)
    rc.display.show_color_image(image_copy)

    if state.turn == 1:
        image_copy = rc_utils.crop(image, CROP_RIGHT[0], CROP_RIGHT[1])
    if state.turn == -1:
        image_copy = rc_utils.crop(image, CROP_LEFT[0], CROP_LEFT[1])
    if state.turn != 0:
        contours_red = rc_utils.find_contours(image_copy, colors['RED'][0], colors['RED'][1])
        contours_green = rc_utils.find_contours(image_copy, colors['GREEN'][0], colors['GREEN'][1])
        contours_blue = rc_utils.find_contours(image_copy, colors['BLUE'][0], colors['BLUE'][1])

        contour_red = rc_utils.get_largest_contour(contours_red, MIN_CONTOUR_AREA)
        contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)
        contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA)

        if contour_red is not None:
            contour_area_red = rc_utils.get_contour_area(contour_red)
        else:
            contour_area_red = 0
        if contour_green is not None:
            contour_area_green = rc_utils.get_contour_area(contour_green)
        else:
            contour_area_green = 0
        if contour_blue is not None:
            contour_area_blue = rc_utils.get_contour_area(contour_blue)
        else:
            contour_area_blue = 0

        if contour_red is not None and contour_area_red > contour_area_green and contour_area_red > contour_area_blue:
            state.color = 'RED'
        elif contour_green is not None and contour_area_green > contour_area_red and contour_area_green > contour_area_blue:
            state.color = 'GREEN'
        elif contour_blue is not None and contour_area_blue > contour_area_red and contour_area_blue > contour_area_green:
            state.color = 'BLUE'
        
        state.turn = 0
    
    if contour_center is not None:
            distance = abs(WIDTH/2 - contour_center[1])
            speed = rc_utils.remap_range(distance, 0, WIDTH/2, 1, 0.5)

            speed = rc_utils.clamp(speed, -1, 1)

    if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, WIDTH, -1, 1)

            angle = rc_utils.clamp(angle, -1, 1)

def follow_lane():
    global image, depth_image, contour_center, contour, speed, angle

    if abs(state.turn < 2):
        image_left = rc_utils.crop(image, (int(HEIGHT*2/3), 0), (HEIGHT, int(WIDTH/2)))
        image_right = rc_utils.crop(image, (int(HEIGHT*2/3), int(WIDTH/2)), (HEIGHT, WIDTH))

        contours_left = rc_utils.find_contours(image_left, colors[state.color][0], colors[state.color][1])
        contours_right = rc_utils.find_contours(image_right, colors[state.color][0], colors[state.color][1])

        contour_left = rc_utils.get_largest_contour(contours_left, MIN_CONTOUR_AREA)
        contour_right = rc_utils.get_largest_contour(contours_right, MIN_CONTOUR_AREA)

        contour_center_left = rc_utils.get_contour_center(contour_left)
        contour_center_right = rc_utils.get_contour_center(contour_right)

        if contour_center_left is not None and contour_center_right is not None:
            row = int((contour_center_left[0] + HEIGHT * 2/3 + contour_center_right[0] + HEIGHT * 2/3) / 2)
            column = int((contour_center_left[1] + contour_center_right[1] + WIDTH / 2) / 2)

            contour_center = (row, column)

            # image_copy = rc_utils.draw_contour(image_copy, contour)
            rc_utils.draw_circle(image, contour_center)
            rc.display.show_color_image(image)

        else:
            # state.color not detected
            image_copy = rc_utils.crop(image, (int(HEIGHT*2/3), 0), (HEIGHT, int(WIDTH)))
            if state.color == 'ORANGE':
                contours = rc_utils.find_contours(image_copy, colors['PURPLE'][0], colors['PURPLE'][1])
            elif state.color == 'PURPLE':
                contours = rc_utils.find_contours(image_copy, colors['ORANGE'][0], colors['ORANGE'][1])
            
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                contour_center_temp = rc_utils.get_contour_center(contour)
                if image_copy is not None:
                    rc_utils.draw_contour(image_copy, contour)
                    rc_utils.draw_circle(image_copy, contour_center_temp)
                    rc.display.show_color_image(image_copy)
                if contour_center_temp[1] < WIDTH/2:
                    state.turn = 2
                else:
                    state.turn = -2
            else:
                state.turn = 0
    
    speed = 1

    if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, WIDTH, -1, 1)
            if state.turn != 0:
                if abs(state.turn == 2): # Sharp turn
                    angle = 1/2 * state.turn
                else:
                    angle = 0.1 * state.turn
                    state.turn = 0

            angle = rc_utils.clamp(angle, -1, 1)

def cone_slalom():
    global image, depth_image, scan, contour_center, contour, speed, angle

    if image is None or depth_image is None or scan is None:
        print("image error")

    # Find next cone color
    if state.color is None:
        contours_red = rc_utils.find_contours(image, colors['RED'][0], colors['RED'][1])
        contours_blue = rc_utils.find_contours(image, colors['BLUE'][0], colors['BLUE'][1])

        contour_red = rc_utils.get_largest_contour(contours_red, MIN_CONTOUR_AREA)
        rc_utils.draw_contour(image, contour_red) if contour_red is not None else None
        contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA)
        rc_utils.draw_contour(image, contour_blue) if contour_blue is not None else None

        # contour_area_red = rc_utils.get_contour_area(contour_red) if contour_red is not None else None
        # contour_area_blue = rc_utils.get_contour_area(contour_blue) if contour_blue is not None else None

        contour_center_red = rc_utils.get_contour_center(contour_red) if contour_red is not None else None
        contour_center_blue = rc_utils.get_contour_center(contour_blue) if contour_blue is not None else None

        cone_distance_red = depth_image[contour_center_red[0]][contour_center_red[1]] if contour_center_red is not None else None
        cone_distance_blue = depth_image[contour_center_blue[0]][contour_center_blue[1]] if contour_center_blue is not None else None

        if (cone_distance_red is None and cone_distance_blue is not None) or (cone_distance_blue is not None and cone_distance_blue < cone_distance_red):
            contour_center = rc_utils.get_contour_center(contour_blue)
            state.color = 'BLUE'

        if (cone_distance_blue is None and cone_distance_red is not None) or (cone_distance_red is not None and cone_distance_red < cone_distance_blue):
            contour_center = rc_utils.get_contour_center(contour_red)
            state.color = 'RED'

    elif state.turn == 0: # Pass cone
        angle = 0
        speed = 1

        closest_cone = rc_utils.get_lidar_closest_point(scan)
        if closest_cone[1] < 999999 and closest_cone[0] > 120 or closest_cone[0] < 240: # Recover
            if state.color == 'RED':
                angle = rc_utils.remap_range(closest_cone[1], 120, 240, -1, 0)
            else:
                angle = rc_utils.remap_range(closest_cone[1], 120, 240, 1, 0)
            # angle = angle if state.color == 'BLUE' else -angle
            state.color = None
            state.turn = None

    # Found cone color: Approach and turn
    elif state.color is not None:
        contours = rc_utils.find_contours(image, colors[state.color][0], colors[state.color][1])

        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        rc_utils.draw_contour(image, contour) if contour is not None else None

        contour_center = rc_utils.get_contour_center(contour) if contour is not None else None
        # print(contour_center)

        cone_distance = depth_image[contour_center[0]][contour_center[1]] if contour_center is not None else None

        closest_cone = rc_utils.get_lidar_closest_point(scan)

        if cone_distance is not None and cone_distance < 150:
            angle = rc_utils.remap_range(contour_center[1], 0, WIDTH, -1, 1)
            if state.color == 'RED':
                # angle = rc_utils.remap_range(contour_center[1]+OFFSET, 0, WIDTH+OFFSET, -1, 1)
                angle += rc_utils.remap_range(cone_distance, 20, 200, 1, 0)
                state.turn = 1
            if state.color == 'BLUE':
                # angle = rc_utils.remap_range(contour_center[1]-OFFSET, 0-OFFSET, WIDTH, -1, 1)
                angle -= rc_utils.remap_range(cone_distance, 20, 200, 1, 0)
                state.turn = -1
            angle = rc_utils.clamp(angle, -1, 1)
        elif closest_cone[1] < 999999 and closest_cone[0] > 60 or closest_cone[0] < 300: # Cone not visible (next to cone)
            print("BANANA")
            state.turn = 0
        else:
            state.turn = None
            angle = rc_utils.remap_range(contour_center[1], 0, WIDTH, -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)
        
        speed = 1

    rc_utils.draw_circle(image, contour_center)
    rc.display.show_color_image(image)

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # rc.drive.set_max_speed(1)

    # Print start message
    print(">> Final Challenge - Time Trial")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global image, depth_image, scan, speed, angle, state

    collect_data()

    set_state()
    print(f'Mode: {state.mode} | Color: {state.color} | Turn: {state.turn}')

    if state.mode == 'line':
        follow_line()
    elif state.mode == 'lane':
        follow_lane()
    elif state.mode == 'slalom':
        cone_slalom()
    elif state.mode == 'wall':
        pass

    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    if (forwardSpeed != 0 or backSpeed != 0):
        speed = forwardSpeed - backSpeed
    
    turnAngle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    if (turnAngle != 0):
        angle = turnAngle

    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle)
    # if image is not None:
    #     rc.display.show_color_image(image)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
