from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh, direction=None):
    sub = yield sub_scripting.get_sub(nh)
    
    yield sub.move.depth(0.75).go()
    
    yield sub.hydrophone_align(25e3)
