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


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    x_location = 0
    y_location = 0

    orientation=Quaternion(x=0, y=0, z=0, w=1)
    yield sub.set_position(sub.pose.position, orientation).go()

    while (x_location < CAMERA_X_CENTER - PIXEL_TOLERANCE) and
          (x_location > CAMERA_X_CENTER + PIXEL_TOLERANCE) and 
          (y_location < CAMERA_Y_CENTER - PIXEL_TOLERANCE) and
          (y_location > CAMERA_Y_CENTER + PIXEL_TOLERANCE):

        center_location = yield sub.get_torpedo_location('center')
        x_location = center_location.x
        y_location = center_location.y

        if x_location > CAMERA_X_CENTER:
            sub.move.right(.1)

        if y_location > CAMERA_Y_CENTER:
            sub.move.up(.1)


    # Still need to figure out how to get ditance from object



