from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    print "lowering impaler"
    yield sub.lower_impaler()
    print "lowered, waiting 3 seconds"
    yield util.sleep(3)
    print "raising impaler"
    yield sub.raise_impaler()
    print "raised impaler"
