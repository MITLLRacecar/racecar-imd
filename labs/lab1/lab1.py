"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
counter = 0
shape = 0
isDriving = False
angle = 0
fspeed = 0
bspeed = 0
turns = 0

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """

    global counter
    global shape
    global isDriving
    global angle
    global fspeed
    global bspeed
    global turns

    counter = 0
    shape = 0
    isDriving = False
    angle = 0
    fspeed = 0
    bspeed = 0
    turns = 0

    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
        "    Y button = drive in a triangle\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global counter
    global shape
    global isDriving
    global angle
    global fspeed
    global bspeed
    global turns

    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    fspeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    bspeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)


    if fspeed > 0:
        rc.drive.set_speed_angle(fspeed, angle)
    if bspeed > 0:
        rc.drive.set_speed_angle(-bspeed, angle)
    if fspeed == 0 and bspeed == 0:
        rc.drive.set_speed_angle(0, angle)

    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle... \n")
        counter = 0
        shape = 1
        isDriving = True

    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square... \n")
        counter = 0
        turns = 0
        shape = 2
        isDriving = True

    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Driving in a figure eight... \n")
        counter = 0
        turns = 0
        shape = 3
        isDriving = True

    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a triangle... \n")
        counter = 0
        turns = 0
        shape = 4
        isDriving = True

    if isDriving:
        counter += rc.get_delta_time()
        if shape == 1: ##circle
            if counter < 5.2:
                rc.drive.set_speed_angle(1, 1)
            elif counter < 6.5:
                rc.drive.set_speed_angle(0, 1)
            else:
                rc.drive.stop()
                isDriving = False

        if shape == 2: #square
            if turns < 4:
                if counter < 1.15:
                    rc.drive.set_speed_angle(1, 0)
                elif counter < 1.4:
                    rc.drive.stop()
                elif counter < 3.8:
                    rc.drive.set_speed_angle(0.5, 1)
                elif counter < 4.8:
                    rc.drive.stop()
                else:
                    counter = 0
                    turns += 1
            else:
                rc.drive.stop()
                isDriving = False

        if shape == 3: #figure eight
            if turns == 0:
                if counter < 2.8:
                    rc.drive.set_speed_angle(1, 0)
                elif counter < 4:
                    rc.drive.stop()
                else:
                    counter = 0
                    turns += 1
            elif turns < 3:
                if counter < 4.2:
                    if turns == 1:
                        rc.drive.set_speed_angle(1, 1)
                    else:
                        rc.drive.set_speed_angle(0.6, -1)
                elif counter < 5.6:
                    rc.drive.stop()
                    if turns == 2:
                        counter = 0
                        turns += 1
                elif counter < 7.7:
                    rc.drive.set_speed_angle(1, 0)
                elif counter < 11.2:
                    rc.drive.stop()
                    counter = 0
                    turns += 1
                else:
                    counter = 0
                    turns += 1
            else:
                rc.drive.stop()
                isDriving = False

        if shape == 4: #triangle
            if turns < 3:
                if counter < 1:
                    rc.drive.set_speed_angle(1, 0)
                elif counter < 3.11:
                    rc.drive.set_speed_angle(1, 1)
                elif counter < 4.81:
                    rc.drive.stop()
                else:
                    counter = 0
                    turns += 1
            else:
                rc.drive.stop()
                isDriving = False

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
