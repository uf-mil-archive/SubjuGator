from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    orig_depth = -sub.pose.position[2]
    
    yield sub.visual_approach_3d('forward', 1.5)
    
    yield sub.move.forward(1.5).go()
    yield sub.move.backward(1.5).go()
    
    yield sub.move.depth(0.5).go()
    yield sub.move.forward(2).go()

    yield sub.move.depth(orig_depth).go()
