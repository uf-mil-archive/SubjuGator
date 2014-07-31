from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh, direction=None, orient_away_from=False, forward=True):
    sub = yield sub_scripting.get_sub(nh)
    
    yield sub.move.depth(0.75).go()
    
    dist = yield sub.get_dvl_range()
    
    print dist
    
    start_pos = sub.pose.position
    if forward:
        fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        yield sub.visual_align('down', 'pipe', dist, selector={
            'left' : sub_scripting.select_by_body_direction([0, +1, 0]),
            'right': sub_scripting.select_by_body_direction([0, -1, 0]),
            None   : lambda items, body_tf: items[0],
        }[direction], orient_away_from=start_pos if orient_away_from else None)
    finally:
        if forward:
            yield fwd_move.cancel()
    
    #yield sub.move.forward(2).go()
