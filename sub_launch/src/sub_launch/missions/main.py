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
from sub_launch.missions import bins

class P(basic.LineOnlyReceiver):
    delimiter = '\n'
    
    def __init__(self, prompt):
        self._prompt = prompt
        self.df = defer.Deferred()
    
    def connectionMade(self):
        self.transport.write(self._prompt)
    
    def lineReceived(self, line):
        self.df.callback(line)
        self.transport.loseConnection()

@util.cancellableInlineCallbacks
def nonblocking_raw_input(prompt):
    p = P(prompt)
    f = stdio.StandardIO(p)
    try:
        res = yield p.df
        defer.returnValue(res)
    finally:
        f.loseConnection()


@util.cancellableInlineCallbacks
def main_list(sub, nh):
    try:
        print 'main start'
	print "starting manipulation"
	#yield manipulation.main(nh)
	print "done manipulation, moving backward"
	#yield sub.move.backward(1).go()
	print "done moving backward, going to path"
	#yield path.main(nh, "right")
	print "done path, going to bins"
	#yield sub.move.forward(2.5).go()
	#yield sub.move.depth(1).go()
	print "at bins, doing bins"
	#yield bins.main(nh)
	print "done bins, going to brunch"
	#yield sub.move.turn_right_deg(90).go()
	#yield sub.move.forward(1.5).go()
	#yield path.main(nh, "right")
	#yield sub.move.forward(3)
	yield path.main(nh, "left")
        yield util.sleep(10)
        print 'main end'
    finally:
        print 'main finally start'
        yield util.sleep(3)
        print 'main finally end'

@util.cancellableInlineCallbacks
def fail_list(sub):
    try:
        print 'fail start'
        yield util.sleep(10)
        print 'fail end'
    finally:
        print 'fail finally'

class TimeoutError(Exception): pass
@util.cancellableInlineCallbacks
def wrap_timeout(df, duration):
    timeout = util.sleep(duration)
    
    try:
        result, index = yield defer.DeferredList([df, timeout], fireOnOneCallback=True, fireOnOneErrback=True)
    finally:
        yield df.cancel()
        yield timeout.cancel()
        df.addErrback(lambda fail: fail.trap(defer.CancelledError))
        timeout.addErrback(lambda fail: fail.trap(defer.CancelledError))
    
    if index == 1:
        raise TimeoutError()
    else:
        defer.returnValue(result)

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh) #None
    
    while True:
        time_left_str = yield nonblocking_raw_input('Enter time left: (e.g. 5:40) ')
        try:
	    if not time_left_str:
		time_left = 60 * 10
		break
            m, s = time_left_str.split(':')
            time_left = 60 * int(m) + int(s)
        except Exception:
            traceback.print_exc()
        else:
            break
    """try:
    	wrap_timeout(get_time_left(time_left), 5)
    except Exception:
        time_left =  60*5+40
    """
    try:
        yield wrap_timeout(main_list(sub, nh), time_left)
    except Exception:
        import traceback
        traceback.print_exc()
    
    yield fail_list(sub)
