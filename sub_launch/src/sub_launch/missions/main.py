from __future__ import division

import traceback

from txros import util
from twisted.internet import threads, stdio, protocol, defer
from twisted.protocols import basic

import sub_scripting


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
def main_list(sub):
    try:
        print 'main start'
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
    sub = None # yield sub_scripting.get_sub(nh)
    
    while True:
        time_left_str = yield nonblocking_raw_input('Enter time left: (e.g. 5:40) ')
        try:
            m, s = time_left_str.split(':')
            time_left = 60 * int(m) + int(s)
        except Exception:
            traceback.print_exc()
        else:
            break
    
    
    try:
        yield wrap_timeout(main_list(sub), 3)
    except Exception:
        import traceback
        traceback.print_exc()
    
    yield fail_list(sub)
