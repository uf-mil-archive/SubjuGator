from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        dist = yield sub.get_dvl_range()
        
        yield sub.visual_align('down', 'wreath', dist)
    finally:
        yield fwd_move.cancel()
    
    orig_depth = -sub.pose.position[2]
    yield sub.move.down(1).go()
    yield sub.move.depth(-0.1).go()
    yield sub.move.depth(orig_depth).go()
