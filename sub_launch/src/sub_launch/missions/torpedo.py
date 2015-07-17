#! usr/bin/env python

from __future__ import division
from txros import util
import sub_scripting
from geometry_msgs.msg import *
import rospy
import math

X_RES = 680
Y_RES = 340
CAMERA_X_CENTER = X_RES/2
CAMERA_Y_CENTER = Y_RES/2
PIXEL_TOLERANCE = 20
MOVE_SCALE = 1
ANGULAR_TOLERANCE = .1

def calc_y_angle(opp_input):
    adjacent = CAMERA_Y_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return .90 - arcsin

def calc_x_angle(opp_input):
    adjacent = CAMERA_X_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return .90 - arcsin

@util.cancellableInlineCallbacks
def center_target(sub):

    while x_location < ANGULAR_TOLERANCE and y_location < ANGULAR_TOLERANCE:

        center_location = yield sub.get_torpedo_location(target)
        x_location = center_location.x
        y_location = center_location.y

        x_move = calc_x_angle(x_location) * MOVE_SCALE
        y_move = calc_y_angle(y_location) * MOVE_SCALE

        if x_location > CAMERA_X_CENTER: 
            print "Moving right", x_move
            yield sub.move.right(x_move)
        if x_location < CAMERA_X_CENTER: 
            print "Moving left", -x_move
            yield sub.move.left(-x_move)
        if y_location > CAMERA_Y_CENTER: 
            print "Moving down", y_move
            yield sub.move.down(y_move)
        if y_location < CAMERA_Y_CENTER:
            print "Moving up", y_move
            yield sub.move.up(y_move)

@util.cancellableInlineCallbacks
def move_to_target(sub):

    TL = yield sub.get_torpedo_location('top_left')
    TL_location = TL.x + TL.y
    yellow_area = yield sub.get_torpedo_location('area')

    if yellow_area > YELLOW_AREA_LIMIT:
        return True
        
    if yellow_area <= YELLOW_AREA_LIMIT:
        yield sub.move.forward(1).go()
        TL = yield sub.get_torpedo_location('top_left')
        TL_location = TL.x + TL.y
        return False


@util.cancellableInlineCallbacks
def main(nh, target = None):

    sub = yield sub_scripting.get_sub(nh)

    if target == None: target = 'center'

    x_location = .5
    y_location = .5

    ready_to_fire = False
    door_open = False

    while not rospy.is_shutdown():

        yield center_target(sub)
        ready_to_fire = yield move_to_target(sub)

        if ready_to_fire == True:
            break

    





