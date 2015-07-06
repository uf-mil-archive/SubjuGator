#! /usr/bin/env python

from __future__ import division
from txros import util
import sub_scripting
import station_hold
from std_msgs.msg import Bool 
import rospy
import sensor_msgs.point_cloud2 as pc2
import math

# 5 degrees - .0425
# 10 degrees - .085

LEFT_TURN = .0425
RIGHT_TURN = -.0425
PIXEL_TOLERANCE = 20
ANGLE_OFFSET = 4
DISPLAY_HEIGHT = 640
DEFAULT_DEVICE = 'delorean'
SUB_LENGTH_OFFSET = 2
MOVE_OFFSET = 2

def calc_angle(opp_input):
    adjacent = DISPLAY_HEIGHT / 2
    opposite = opp_input
    hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
    arcsin = math.asin(opposite/hypotenuse)
    return arcsin

def find_target():

    # While the sub is not locked on to the target's orientation
    while True:
        # Get new target location - 
        # 0 is dead center
        # positive values mean right
        # negative values mean left
        msg =  yield sub.get_target_location(target)
        x_pixel, y_pixel, orientation = msg[0], msg[1], msg[2] 


        # Throws out bad readings
        if x_pixel != 0 and y_pixel != 0 and orientation != 0:

             print x_pixel, y_pixel, orientation

            # If the pixel location is within our range of error
            if abs(msg) < PIXEL_TOLERANCE:
                print "circle in Center at location: " + str(msg) + " --- Locking Target"
                break
                # Break the loop and continue

            angle_move = calc_angle(msg) / ANGLE_OFFSET

            # If the target is right of the center 
            if msg > 0:
                # turn the number of degrees right between the center and the target, minus an offset
                #yield sub.move.turn_left(angle_move).go()
                print "Turning right", abs(angle_move)

            # If the target is left of the center
            if msg < 0:
                # turn the number of degrees left between the center and the target, minus an offset
                #yield sub.move.turn_left(-angle_move).go()
                print "Turning left", abs(angle_move)

            print target + " at pixel location: ", abs(msg)

@util.cancellableInlineCallbacks
def main(nh, target=None):

    if target == None:
        target = DEFAULT_DEVICE

    sub = yield sub_scripting.get_sub(nh, False, False)

    # While the sub is still distant from the target
    while True and not rospy.is_shutdown():

        temp_distance = 0
        avg_distance = 0
        shortest_distance = 100
        farthest_distance = 0

        find_target(target)

        '''

        avg_distance = temp_distance/len(hold) - sub_LENGTH_OFFSET
        shortest_distance = shortest_distance/MOVE_OFFSET - sub_LENGTH_OFFSET
        farthest_distance = farthest_distance - sub_LENGTH_OFFSET

        print "Average distance from target:", avg_distance 
        print "Shortest distance between sub and object:", shortest_distance 
        print "Farther distance between sub and object:", farthest_distance 

        final_move = shortest_distance

        if farthest_distance > 3.5: 
            input("Press enter to move forward" + str(final_move) + " meters")
            # Print only if a move is commanded
            print "Moving forward " + str(final_move) + " meters"
            yield sub.move.forward(final_move).go()
        if farthest_distance <= 2: 
            input("Press enter to move forward one meter")
            # Print only if a move is commanded
            print "Moving forward one meter"
            yield sub.move.forward(1).go()
            yield sub.move.forward(-6).go()
            print 'Find target success'
            break

        # delete variables to avoid any threading problems
        del avg_distance, temp_distance, farthest_distance, shortest_distance
        '''