from __future__ import division

import traceback
import numpy

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
    yield sub.move.depth(1).go()
    yield sub.move.forward(15).go()
    m = numpy.array([7.6448368633, 41.202367025499996, 0]) * .8
    yield sub.move.look_at_rel_without_pitching(m).go()
    yield sub.move.absolute(m).go()

@util.cancellableInlineCallbacks
def fail_list(nh):
    sub = yield sub_scripting.get_sub(nh)
    yield recovery.main(nh)

@util.cancellableInlineCallbacks
def main(nh):
    #while True:
    #    time_left_str = yield util.nonblocking_raw_input('Enter time left: (e.g. 5:40) ')
    #    try:
    #        m, s = time_left_str.split(':')
    #        time_left = 60 * int(m) + int(s)
    #    except Exception:
    #        traceback.print_exc()
    #    else:
    #        break
    
    sub = yield sub_scripting.get_sub(nh)
    
    try:
        yield util.wrap_timeout(main_list(nh), 11*60)
    except Exception:
        traceback.print_exc()
    
    yield fail_list(nh)


