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
MOVE_SCALE = 1
PIXEL_TOLERANCE = 20
ANGULAR_TOLERANCE = .03
ORIENTATION_TOLERANCE = .03
DOWN_MOVEMENT = .05

@util.cancellableInlineCallbacks
def move_down(sub):

    dist_from_ground = yield sub.get_dvl_range()

    while dist_from_ground > 1:
        yield sub.move.down(DOWN_MOVEMENT).go()


@util.cancellableInlineCallbacks
def main(nh, target = None, orient_target = None):
    sub = yield sub_scripting.get_sub(nh)

    align_target = target
    if align_target == None: align_target = 'vis_simulator'
    orientation_align_target = orient_target
    if orientation_align_target == None: orientation_align_target = 'delorean'

    x_location = 0
    y_location = 0

    while not rospy.is_shutdown():

        yield sub.orient_and_align(align_target, orientation_align_target, MOVE_SCALE, ANGULAR_TOLERANCE, ORIENTATION_TOLERANCE)
        #yield sub.align(align_target, MOVE_SCALE, ANGULAR_TOLERANCE)

    # Still need to figure out how to get ditance from object



