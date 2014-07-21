from __future__ import division

from txros import util
from main import wrap_timeout, TimeoutError

import math
import sub_scripting
import numpy

select_centered = lambda objs, body_tf: min(objs, key=lambda obj: math.sqrt(float(obj['center'][0])**2 + float(obj['center'][1])**2))

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
        yield sub.visual_align('down', 'wreath/moonrock', dist, selector=select_by_body_direction([0,1,0]))
    finally:
        yield fwd_move.cancel()       
    
    """while True:
        dist = yield sub.get_dvl_range()
        yield sub.move.down(dist-1.75).go()
        dist = yield sub.get_dvl_range()
        try:
            yield wrap_timeout(sub.visual_align('down', 'wreath/cheese', dist-.3, selector=select_by_body_direction([0,1,0])), 20)
        except TimeoutError:
            print 'timed out'
            break
        print 'didnt time out'
        try:
            yield sub.lower_impaler()
            print 'about to move relative'
            yield sub.move.relative([-.07,-.08,0]).go()
            print 'about to move down'
            yield sub.move.down(1.05).go(speed=.1)
            try:
                yield sub.expand_impaler()
                print 'moving back up'
                yield sub.move.depth(.5).go()
                print 'moving to original depth'
                yield sub.move.depth(orig_depth).go()
            finally:
                yield sub.contract_impaler()
                yield util.sleep(5)
        finally:
            yield sub.raise_impaler()"""
    try:
        while True:
            dist = yield sub.get_dvl_range()
            yield sub.move.down(dist-1.75).go()
            dist = yield sub.get_dvl_range()
            try:
                yield wrap_timeout(sub.visual_align('down', 'wreath/moonrock', dist-.3, selector=select_centered), 20)
            except TimeoutError:
                print 'timed out'
                break
            print "relative"
            yield sub.move.relative([-.07,-.08,0]).go()
            print "down"
            x = sub.move.down(1.05).go(speed=.2)
            yield util.sleep(2)
            yield sub.lower_impaler()
            yield x
            print "expand"
            yield sub.expand_impaler()
            yield sub.move.depth(.8).go()
            y = sub.move.down(.7).go()
            yield util.sleep(3.5)
            yield sub.contract_impaler()
            yield sub.raise_impaler()
            yield y
    finally:
        yield sub.contract_impaler()
        yield sub.raise_impaler()
        yield sub.move.depth(orig_depth).go()
