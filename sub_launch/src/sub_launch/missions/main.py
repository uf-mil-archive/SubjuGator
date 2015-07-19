from __future__ import division

import traceback
import numpy
from txros import util
from twisted.internet import threads, stdio, protocol, defer
from twisted.protocols import basic
import sub_scripting
from sub_launch.missions import green_buoy, yellow_buoy, red_buoy, torpedo, path

ONE_MINUTE = 60
TOTAL_TIME = ONE_MINUTE * 20
BUOY_TIME = ONE_MINUTE * 3
TORPEDO_TIME = ONE_MINUTE * 2
PATH_TIME = ONE_MINUTE / 2

@util.cancellableInlineCallbacks
def inch_forward(nh, sub):

    sub.change_current_vision(False,False,False,False,True,False)

    while: # PUT CONDITION OF SEEING NEXT ORANGE OBJECT
        yield sub.move.forward(1).go()

@util.cancellableInlineCallbacks
def buoys(nh, sub):

    sub.change_current_vision(False,False,False,False,False,True)

    pos = yield sub.pose.position
    print pos
    
    try:
        yield util.wrap_timeout(yellow_buoy.main(nh), BUOY_TIME / 3)
    except Exception:
        traceback.print_exc()

    yield sub.move.set_position(pos).go()

    try:
        yield util.wrap_timeout(red_buoy.main(nh), BUOY_TIME / 3)
    except Exception:
        traceback.print_exc()

    yield sub.move.set_position(pos).go()

    try:
        yield util.wrap_timeout(green_buoy.main(nh), BUOY_TIME / 3)
    except Exception:
        traceback.print_exc()


@util.cancellableInlineCallbacks
def torpedos(nh):

    sub.change_current_vision(False,True,True,False,False,False)

    try:
        yield util.wrap_timeout(torpedo.main(nh), TORPEDO_TIME / 2)
    except Exception:
        traceback.print_exc()


@util.cancellableInlineCallbacks
def main_list(nh):

    '''

    Wrap individual tasks in their own timeouts so we can assign time to 
    the task and any support moves needed that are not included in the tasks
    mission main function

    '''

    sub = yield sub_scripting.get_sub(nh)
    yield sub.move.depth(1).go()

    try:
        yield util.wrap_timeout(buoys(nh, sub), BUOY_TIME)
    except Exception:
        traceback.print_exc()
    finally:
        sub.change_current_vision(False,False,False,False,False,False)


    try:
        sub.change_current_vision(False,False,False,False,True,False)
        yield util.wrap_timeout(path.main(nh), PATH_TIME)
    except Exception:
        traceback.print_exc()
    finally:
        sub.change_current_vision(False,False,False,False,False,False)

    try:
        yield util.wrap_timeout(torpedos(nh), TORPEDO_TIME)
    except Exception:
        traceback.print_exc()
    finally:
        sub.change_current_vision(False,False,False,False,False,False)


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    '''

    Vision order:
        recovery_vision
        torpedo_vision
        torpedo_area_vision
        train_vision
        path_vision
        buoys_vision

    '''

    sub.change_current_vision(False,False,False,False,False,False)
    
    # WRAP ENTIRE MISSION IN TIMEOUT
    try:
        yield util.wrap_timeout(main_list(nh), TOTAL_TIME)
    except Exception:
        traceback.print_exc()



