from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    yield sub.visual_align('down', 'pipe')
    
    yield sub.move.forward(1).go()
