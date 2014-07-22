from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    #yield sub.lower_down_grabber()
    while True:
        yield util.sleep(2)
        yield sub.open_down_grabber()
        yield util.sleep(2)
        yield sub.close_down_grabber()
    #yield util.sleep(2)
    #yield sub.close_down_grabber()
    #yield util.sleep(2)
    #yield sub.raise_down_grabber()
