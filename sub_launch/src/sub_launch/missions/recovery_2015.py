#! usr/bin/env python

from __future__ import division
from txros import util
import sub_scripting
from geometry_msgs.msg import *

X_RES = 640
Y_RES = 480
CAMERA_X_CENTER = X_RES/2
CAMERA_Y_CENTER = Y_RES/2

PIXEL_TOLERANCE = 20

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

    if target = None: target = 'delorean'

    x_location = 0
    y_location = 0

    orientation=Quaternion(x=0, y=0, z=0, w=1)
    yield sub.set_position(sub.pose.position, orientation).go()

    while (x_location < CAMERA_X_CENTER - PIXEL_TOLERANCE) and
          (x_location > CAMERA_X_CENTER + PIXEL_TOLERANCE) and 
          (y_location < CAMERA_Y_CENTER - PIXEL_TOLERANCE) and
          (y_location > CAMERA_Y_CENTER + PIXEL_TOLERANCE):

        center_location = yield sub.get_target_location(target)
        x_location = center_location.x
        y_location = center_location.y
        orientation = center_location.z
        print oientation

        x_angle = calc_x_angle(x_location)
        y_angle = calc_y_angle(y_location)

        print x_angle, y_angle

        if x_location > CAMERA_X_CENTER:
            sub.move.right(.1)

        if y_location > CAMERA_Y_CENTER:
            sub.move.up(.1)

        if x_location < CAMERA_Y_CENTER:
            sub.move.right(-.1)

        if y_location < CAMERA_Y_CENTER:
            sub.move.down(-.1) 

        '''

    TL = yield sub.get_torpedo_location('top_left')
    TL_location = TL.x + TL.y

    while TL_location > 50:
        yield sub.move.forward(1).go()
        TL = yield sub.get_torpedo_location('top_left')
        TL_location = TL.x + TL.y

    # Still need to figure out how to get ditance from object



