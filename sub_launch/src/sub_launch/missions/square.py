from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    for i in xrange(4):
        print 'Side', i
        yield sub.move.forward(2)
        yield sub.move.turn_left_deg(90)
