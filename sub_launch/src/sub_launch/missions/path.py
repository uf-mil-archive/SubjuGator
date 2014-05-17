from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    dist = yield sub.get_dvl_range()
    
    print dist
    
    sub.move.go(linear=[0.25, 0, 0])
    
    yield sub.visual_align('down', 'pipe', dist)
    
    yield sub.move.forward(2).go()
