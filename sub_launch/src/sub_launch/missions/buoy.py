from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    yield sub.visual_approach_3d('forward', 1.5)
    
    yield sub.move.forward(1.5).go()
    yield sub.move.backward(1.5).go()
    
    yield sub.move.depth(0.5).go()
