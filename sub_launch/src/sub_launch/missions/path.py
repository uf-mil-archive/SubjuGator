from __future__ import division

import numpy

from txros import util

import sub_scripting


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
def main(nh, direction=None):
    sub = yield sub_scripting.get_sub(nh)
    
    dist = yield sub.get_dvl_range()
    
    print dist
    
    sub.move.go(linear=[0.25, 0, 0])
    
    yield sub.visual_align('down', 'pipe', dist, selector={
        'left' : select_by_body_direction([0, +1, 0]),
        'right': select_by_body_direction([0, -1, 0]),
        None   : lambda objs: objs[0],
    }[direction])
    
    yield sub.move.forward(2).go()
