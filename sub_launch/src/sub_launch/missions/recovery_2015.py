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
ANGULAR_TOLERANCE = .03
ORIENTATION_TOLERANCE = .03

def calc_y_angle(opp_input):
    adjacent = CAMERA_Y_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return arcsin

def calc_x_angle(opp_input):
    adjacent = CAMERA_X_CENTER
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.acos(opposite/hypotenuse)
    return arcsin

@util.cancellableInlineCallbacks
def main(nh, target = None):
    sub = yield sub_scripting.get_sub(nh)

    if target == None: target = 'delorean'

    x_location = 0
    y_location = 0


    while not rospy.is_shutdown():

        center_location = yield sub.get_target_location(target)
        x_location = center_location.x
        y_location = center_location.y
        orientation = center_location.z
        print orientation

        x_angle = calc_x_angle(x_location)
        y_angle = calc_y_angle(y_location)

        print x_angle, y_angle

        if x_location < ANGULAR_TOLERANCE and y_location < ANGULAR_TOLERANCE and orientation < ORIENTATION_TOLERANCE: break

        if x_location > CAMERA_X_CENTER:
            sub.move.right(.1).go()

        if y_location > CAMERA_Y_CENTER:
            sub.move.left(-.1).go()

        if x_location > CAMERA_X_CENTER:
            sub.move.right(.1).go()

        if y_location > CAMERA_Y_CENTER:
            sub.move.left(-.1).go()

        
        if 


    # Still need to figure out how to get ditance from object



