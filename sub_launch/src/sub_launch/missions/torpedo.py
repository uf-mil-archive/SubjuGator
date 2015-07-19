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
ready_to_fire = False


@util.cancellableInlineCallbacks
def move_to_target(sub):

    global ready_to_fire

    TL = yield sub.get_target_location('top_left')
    TL_location = TL.x + TL.y
    yellow_area = yield sub.get_target_location('area')
        
    if yellow_area <= YELLOW_AREA_LIMIT:
        yield sub.move.forward((YELLOW_AREA_LIMIT - yellow_area)*FORWARD_MOVE_SCALE).go()
        TL = yield sub.get_target_location('top_left')
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

        yield sub.align(align_target, MOVE_SCALE, ANGULAR_TOLERANCE)
        yield move_to_target(sub)

        if ready_to_fire == True:
            break

    #yield sub.

    #yield sub.fire_left_torpedo()

    # Need to measure the torpedo target to get distances to dead recon from here

    #yield sub.move.forward(1).go()

    





