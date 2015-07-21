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

    while dist_from_ground > .5:
        yield sub.move.down(DOWN_MOVEMENT).go()


@util.cancellableInlineCallbacks
def main(nh, target = None, orient_target = None):
    sub = yield sub_scripting.get_sub(nh)
    picked_up_target = False

    align_target = target
    if align_target == None: align_target = 'delorean'
    orientation_align_target = orient_target
    if orientation_align_target == None: orientation_align_target = 'delorean'

    x_location = 0
    y_location = 0

    while not rospy.is_shutdown():

        yield sub.orient_and_align(align_target, orientation_align_target, MOVE_SCALE, ANGULAR_TOLERANCE, ORIENTATION_TOLERANCE)
    # Move down until distance from obbject is 1 meter
    yield move_down(sub)

    while picked_up_target == False:
        yield sub.close_gripper()
        yield sub.move.up(1).go()
        # Scan ten times to verify that we have picked up object
        for x in xrange(1,10):
            target_scan = yield sub.get_target_location(align_target)
            yield util.sleep(.1)
            if target_scan.x == 0 and target_scan.y == 0: picked_up_target = True
    








