from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    sub.move.go(linear=[0.25, 0, 0])
    
    yield sub.visual_align('down', 'wreath')
    
    orig_depth = -sub._pose.position[2] # XXX don't use private attr
    yield sub.move.down(1).go()
    yield sub.move.depth(-0.1).go()
    yield sub.move.depth(orig_depth).go()
