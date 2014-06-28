from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    print "firing left torpedo"
    yield sub.fire_left_torpedo()
    print "fired, waiting 3 seconds"
    yield util.sleep(3)
    print "firing right torpedo"
    yield sub.fire_right_torpedo()
    print "fired right torpedo"
