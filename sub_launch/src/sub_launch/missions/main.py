from __future__ import division

import traceback
import numpy
from txros import util
from twisted.internet import threads, stdio, protocol, defer
from twisted.protocols import basic
import sub_scripting
from sub_launch.missions import buoys, torpedo, path
from sub8_vision_arbiter import *

ONE_MINUTE = 60
TOTAL_TIME = ONE_MINUTE * 15
BUOY_TIME = ONE_MINUTE * 2
TORPEDO_TIME = ONE_MINUTE * 2
PATH_TIME = ONE_MINUTE / 2
PORTAL_TIME = ONE_MINUTE * 2

@util.cancellableInlineCallbacks
def buoy(nh):

    #sub.change_current_vision(False,False,False,False,False,True)

    try:
        yield util.wrap_timeout(buoys.main(nh), BUOY_TIME)
    except Exception:
        traceback.print_exc()


@util.cancellableInlineCallbacks
def torpedos(nh):

    #sub.change_current_vision(False,True,True,False,False,False)

    try:
        yield util.wrap_timeout(torpedo.main(nh), TORPEDO_TIME / 2)
    except Exception:
        traceback.print_exc()


@util.cancellableInlineCallbacks
def main_list(nh,sub):

    '''

    Wrap individual tasks in their own timeouts so we can assign time to 
    the task and any support moves needed that are not included in the tasks
    mission main function

    '''

    yield sub.move.depth(1).go()
    yield sub.move.forward(10).go()

    try:
        yield util.wrap_timeout(buoy(nh), BUOY_TIME)
    except Exception:
        traceback.print_exc()
    finally: pass
        #sub.change_current_vision(False,False,False,False,False,False)

    try:
        #sub.change_current_vision(False,False,False,False,True,False)
        yield util.wrap_timeout(path.main(nh), PATH_TIME)
    except Exception:
        traceback.print_exc()
    finally: pass

    yield sub.move.forward(3).go()
    try:
        #sub.change_current_vision(False,False,False,False,True,False)
        yield util.wrap_timeout(portal.main(nh), PORTAL_TIME)
    except Exception:
        traceback.print_exc()
    finally: pass
        #sub.change_current_vision(False,False,False,False,False,False)

    try:
        #sub.change_current_vision(False,False,False,False,True,False)
        yield util.wrap_timeout(path.main(nh), PATH_TIME)
    except Exception:
        traceback.print_exc()
    finally: pass
        #sub.change_current_vision(False,False,False,False,False,False)

    yield sub.move.forward(3).go()

    try:
        yield util.wrap_timeout(torpedos(nh), TORPEDO_TIME)
    except Exception:
        traceback.print_exc()
    finally: pass
        #sub.change_current_vision(False,False,False,False,False,False)


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    begin = False 

    while True:
        begin = yield sub.get_begin()
        if begin == True:
            break

    print "Starting mission"

    '''

    Vision order:
        recovery_vision
        torpedo_vision
        torpedo_area_vision
        train_vision
        path_vision
        buoys_vision

    '''

    #sub.change_current_vision(False,False,False,False,False,False)

    # WRAP ENTIRE MISSION IN TIMEOUT
    try:
        yield util.wrap_timeout(main_list(nh, sub), TOTAL_TIME)
    except Exception:
        traceback.print_exc()



