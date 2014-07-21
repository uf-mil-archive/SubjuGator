from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        yield sub.visual_approach('forward', 'hedge', size_estimate=4*12*.0254, desired_distance=2)
    finally:
        yield fwd_move.cancel()
    
    yield sub.move.right(.6).go()
    yield sub.move.forward(3.5).go()
    yield sub.move.left(1.2).go()
    yield sub.move.backward(2.25).go()
    yield sub.move.right(1.2).go()
    yield sub.move.forward(3.5).go()
