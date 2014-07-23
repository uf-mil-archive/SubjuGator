from __future__ import division

import traceback

from txros import util
from twisted.internet import threads, stdio, protocol, defer
from twisted.protocols import basic

import sub_scripting

from sub_launch.missions import shooter
from sub_launch.missions import buoy
from sub_launch.missions import path
from sub_launch.missions import manipulation
from sub_launch.missions import bins, maneuvering, recovery


@util.cancellableInlineCallbacks
def main_list(nh):
    sub = yield sub_scripting.get_sub(nh)
    try:
        print 'starting buoy'
        yield buoy.main(nh)
        print 'starting path'
        yield path.main(nh)
        print 'starting maneuvering'
        yield maneuvering.main(nh)
        print 'starting left path'
        yield path.main(nh, 'left')
        print 'staring bins'
        stored_pose1 = sub.pose
        yield bins.main(nh)
        stored_pose2 = sub.pose
        print 'going back'
        yield sub.move.set_position(stored_pose1.position).set_orientation(stored_pose1.orientation).go()
        print 'starting shooter'
        yield shooter.main(nh)
        print 'starting path'
        yield sub.move.set_position(stored_pose2.position).set_orientation(stored_pose1.orientation).go()
        yield sub.move.turn_right_deg(90).go()
        yield sub.move.forward(1.75).go()
        yield path.main(nh)
        print 'staring manipulation'
        yield manipulation.main(nh)
        yield sub.move.depth(0.75).turn_right_deg(20).go()
        yield sub.move.forward(6).go()
        yield recovery.main(nh)
    finally:
        print 'main finally start'
        #yield util.sleep(3)
        print 'main finally end'

@util.cancellableInlineCallbacks
def fail_list(nh):
    sub = yield sub_scripting.get_sub(nh)
    try:
        print 'fail start'
        #yield sub.move.depth(0).go()
        print 'fail end'
    finally:
        print 'fail finally'

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    while True:
        time_left_str = yield util.nonblocking_raw_input('Enter time left: (e.g. 5:40) ')
        try:
            m, s = time_left_str.split(':')
            time_left = 60 * int(m) + int(s)
        except Exception:
            traceback.print_exc()
        else:
            break
    
    try:
        yield util.wrap_timeout(main_list(nh), time_left - 60)
    except Exception:
        traceback.print_exc()
    
    yield fail_list(nh)
