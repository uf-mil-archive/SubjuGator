from __future__ import division

from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)

    while True:
        raw_input("Press enter for sub location...")
        pose = yield sub.pose.position
        print pose
