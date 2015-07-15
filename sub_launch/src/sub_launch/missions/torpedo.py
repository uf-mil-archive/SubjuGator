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

MOVE_SCALE = 2
ANGULAR_TOLERANCE = .03

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
def main(nh, target = None):

    sub = yield sub_scripting.get_sub(nh)

    if target == None: target = 'center'

    x_location = 0
    y_location = 0

    orientation=Quaternion(x=0, y=0, z=0, w=1)
    #yield sub.set_position(sub.pose.position, orientation).go()

    while not rospy.is_shutdown():

        center_location = yield sub.get_torpedo_location(target)
        x_location = center_location.x
        y_location = center_location.y

        x_move = calc_x_angle(x_location) * MOVE_SCALE
        y_move = calc_y_angle(y_location) * MOVE_SCALE

        if x_location < ANGULAR_TOLERANCE and y_location < ANGULAR_TOLERANCE: break

        if x_location > CAMERA_X_CENTER: 
            print "Moving right", x_move
            yield sub.move.right(x_move)
        if x_location < CAMERA_X_CENTER: 
            print "Moving left", x_move
            yield sub.move.left(x_move)
        if y_location > CAMERA_Y_CENTER: 
            print "Moving down", y_move
            yield sub.move.down(y_move)
        if y_location < CAMERA_Y_CENTER:
            print "Moving up", y_move
            yield sub.move.up(y_move)

        print 
        yield util.sleep(.3)


        '''

    TL = yield sub.get_torpedo_location('top_left')
    TL_location = TL.x + TL.y

    while TL_location > 50:
        yield sub.move.forward(1).go()
        TL = yield sub.get_torpedo_location('top_left')
        TL_location = TL.x + TL.y

    # Still need to figure out how to get ditance from object

    '''



