from __future__ import division

from txros import util
from main import wrap_timeout, TimeoutError

import sub_scripting
import numpy

def select_by_body_direction(body_vector):
    body_vector = numpy.array(body_vector)
    def _(results, body_tf):
        def get_wantedness(result):
            pos_vec = numpy.array(map(float, result['center']))
            pos_vec_body = body_tf.transform_vector(pos_vec)
            return pos_vec_body.dot(body_vector)
        return max(results, key=get_wantedness)
    return _

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        dist = yield sub.get_dvl_range()
    
        orig_depth = -sub.pose.position[2]
        
        dist = yield sub.get_dvl_range()
        yield sub.move.down(dist-1.75).go()       
        yield sub.visual_align('down', 'wreath/cheese', dist, selector=select_by_body_direction([0,1,0]))
    finally:
        yield fwd_move.cancel()
        
    
    while True:
        dist = yield sub.get_dvl_range()
        yield sub.move.down(dist-1.75).go()
        dist = yield sub.get_dvl_range()
        try:
            yield wrap_timeout(sub.visual_align('down', 'wreath/cheese', dist-.3, selector=select_by_body_direction([0,1,0])), 20)
        except TimeoutError:
            print 'timed out'
            break
        print 'didnt time out'
        yield sub.lower_impaler()
        print 'about to move relative'
        yield sub.move.relative([-.1,-.1,0]).go()
        print 'about to move down'
        yield sub.move.down(1).go()
        print 'moving back up'
        yield sub.move.depth(.5).go()
        print 'impaler'
        yield sub.raise_impaler()
        print 'moving to original depth'
        yield sub.move.depth(orig_depth).go()
    
    while True:
        dist = yield sub.get_dvl_range()
        yield sub.move.down(dist-1.75).go()
        try:
            yield wrap_timeout(sub.visual_align('down', 'wreath/moonrock', dist, selector=select_by_body_direction([0,1,0])), 20)
        except TimeoutError:
            break
        yield sub.lower_impaler()
        yield sub.move.relative([-.1,-.1,0])
        yield sub.move.down(1).go()
        yield sub.move.depth(.5).go()
        yield sub.raise_impaler()
        yield sub.move.depth(orig_depth).go()
