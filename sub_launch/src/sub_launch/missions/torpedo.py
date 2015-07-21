#! usr/bin/env python

from __future__ import division
from txros import util
import sub_scripting
from geometry_msgs.msg import *
import rospy
import math


PIXEL_TOLERANCE = 20
MOVE_SCALE = 1
ANGULAR_TOLERANCE = .05
YELLOW_AREA_LIMIT = .43
FORWARD_MOVE_SCALE = 10
aligned_to_target = False
centered_pos = 0
door_pos = 0
quadrant = 0
CAMERA_X_CENTER = 640/2
CAMERA_Y_CENTER = 480/2

def solve_handle_local():

    global quadrant
    x_location = quadrant.x
    y_location = quadrant.y

    if x_location < CAMERA_X_CENTER and y_location < CAMERA_Y_CENTER: return 2
    if x_location > CAMERA_X_CENTER and y_location < CAMERA_Y_CENTER: return 1
    if x_location < CAMERA_X_CENTER and y_location < CAMERA_Y_CENTER: return 3
    if x_location < CAMERA_X_CENTER and y_location < CAMERA_Y_CENTER: return 4


@util.cancellableInlineCallbacks
def open_door():

    global centered_pos
    global door_pos
    global quadrant
    # Move to center position
    yield sub.move.set_position(centered_pos).go()
    # Get position of handle
    quadrant = yield sub.get_target_location('handle')
    # Align sub to handle
    yield sub.align('handle', MOVE_SCALE, ANGULAR_TOLERANCE)
    # Grab position of sub in front of handle
    door_pos = yield sub.pose.position
    # Move door to right
    yield sub.move.forward(1).go()
    yield sub.move.right(1).go()
    # Move back to center
    yield sub.move.set_position(centered_pos).go()
    # Check if handle is still visible
    handle_check = yield sub.get_target_location('handle')
    # If it is visible, run again
    if handle_check.x != 0 and handle_check.y != 0: open_door()

@util.cancellableInlineCallbacks
def move_to_target(sub):

    TL = yield sub.get_target_location('top_left')
    TL_location = TL.x + TL.y
    yellow_area = yield sub.get_target_location('area')
        
    while yellow_area <= YELLOW_AREA_LIMIT:
        yield sub.move.forward((YELLOW_AREA_LIMIT - yellow_area)*FORWARD_MOVE_SCALE).go()
        yellow_area = yield sub.get_target_location('area')

@util.cancellableInlineCallbacks
def main(nh):

    sub = yield sub_scripting.get_sub(nh)s

    global door_pos
    global centered_pos

    # Align sub to center of target
    yield sub.align('center', MOVE_SCALE, ANGULAR_TOLERANCE)
    # Move sub until it is one meter away from target
    yield move_to_target(sub)
    # Grab centered position of the sub
    centered_pos = yield sub.pose.position

    yield open_door()
    # Move back to position of door 
    yield sub.move.set_position(door_pos).go()
    # Fire first torpedo
    yield sub.fire_left_torpedo()

    handle_local = solve_handle_local()

    if handle_local == 1: yield sub.move.down(1).go()
    if handle_local == 2: yield sub.move.right(1).go()
    if handle_local == 3: yield sub.move.up(1).go()
    if handle_local == 4: yield sub.move.left(1).go()

    yield sub.fire_right_torpedo()

    





