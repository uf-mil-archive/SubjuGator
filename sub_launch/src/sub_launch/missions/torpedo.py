#! usr/bin/env python

from __future__ import division
from txros import util
import sub_scripting
from geometry_msgs.msg import *
import rospy
import math

X_RES = 640
Y_RES = 480
CAMERA_X_CENTER = X_RES/2
CAMERA_Y_CENTER = Y_RES/2
PIXEL_TOLERANCE = 20
MOVE_SCALE = 1
ANGULAR_TOLERANCE = .05
YELLOW_AREA_LIMIT = .43
FORWARD_MOVE_SCALE = 10

ready_to_fire = False

def calc_y_angle(opp_input):
    adjacent = CAMERA_Y_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return abs(.80 - arcsin)

def calc_x_angle(opp_input):
    adjacent = CAMERA_X_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return abs(.80 - arcsin)

@util.cancellableInlineCallbacks
def center_target(sub):

    global align_target

    x_location = 1
    y_location = 1

    while x_location > ANGULAR_TOLERANCE and y_location > ANGULAR_TOLERANCE:

        center_location = yield sub.get_torpedo_location(align_target)
        x_location = center_location.x
        y_location = center_location.y

        print x_location, y_location

        x_move = calc_x_angle(x_location) * MOVE_SCALE
        y_move = calc_y_angle(y_location) * MOVE_SCALE

        if x_location < CAMERA_X_CENTER: 
            print "Moving right", x_move
            yield sub.move.right(x_move)
        if x_location > CAMERA_X_CENTER: 
            print "Moving left", x_move
            yield sub.move.left(x_move)
        if y_location < CAMERA_Y_CENTER: 
            print "Moving down", y_move
            yield sub.move.down(y_move)
        if y_location > CAMERA_Y_CENTER:
            print "Moving up", y_move
            yield sub.move.up(y_move)

@util.cancellableInlineCallbacks
def move_to_target(sub):

    global ready_to_fire

    TL = yield sub.get_torpedo_location('top_left')
    TL_location = TL.x + TL.y
    yellow_area = yield sub.get_torpedo_location('area')
        
    if yellow_area <= YELLOW_AREA_LIMIT:
        yield sub.move.forward((YELLOW_AREA_LIMIT - yellow_area)*FORWARD_MOVE_SCALE).go()
        TL = yield sub.get_torpedo_location('top_left')
        TL_location = TL.x + TL.y

    if yellow_area > YELLOW_AREA_LIMIT:
        ready_to_fire = True


@util.cancellableInlineCallbacks
def main(nh, target = None):

    sub = yield sub_scripting.get_sub(nh)


    global ready_to_fire
    global align_target
    align_target = target
    if align_target == None: align_target = 'vis_simulator'

    door_open = False

    while not rospy.is_shutdown():

        yield center_target(sub)
        yield move_to_target(sub)

        if ready_to_fire == True:
            break

    

    # Need to measure the torpedo target to get distances to dead recon from here

    #yield sub.move.forward(1).go()

    





