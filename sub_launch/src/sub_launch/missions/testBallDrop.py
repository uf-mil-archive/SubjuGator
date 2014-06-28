from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    print "dropping ball"
    yield sub.drop_ball()
    print "ball dropped"
    #yield util.sleep(3)
