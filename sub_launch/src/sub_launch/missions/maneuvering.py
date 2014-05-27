from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    sub.move.go(linear=[0.25, 0, 0])
    
    yield sub.visual_approach('forward', 'hedge', size_estimate=4*12*.0254, desired_distance=2)
    
    yield sub.move.right(.6).go()
    yield sub.move.forward(3.5).go()
    yield sub.move.left(1.2).go()
    yield sub.move.backward(3.5).go()
    yield sub.move.right(1.2).go()
    yield sub.move.forward(5).go()
