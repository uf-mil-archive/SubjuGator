from __future__ import division

from txros import util
import numpy
import sub_scripting

def select_by_body_direction(body_vector):
    body_vector = numpy.array(body_vector)
    def _(results, body_tf):
        min_radius = min(numpy.linalg.norm(map(float, result['direction'])) for result in results)
        results = [result for result in results if numpy.linalg.norm(map(float, result['direction'])) < 1.5 * min_radius]
        def get_wantedness(result):
            pos_vec = numpy.array(map(float, result['center']))
            pos_vec_body = body_tf.transform_vector(pos_vec)
            return pos_vec_body.dot(body_vector)
        
        return max(results, key=get_wantedness)
    return _

@util.cancellableInlineCallbacks
def main(nh):
    print "starting shooter"
    sub = yield sub_scripting.get_sub(nh)
    
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        yield sub.visual_approach('forward', 'shooter', size_estimate=5*.0254, desired_distance=1.5, selector=select_by_body_direction([0,1,0]))
    finally:
        yield fwd_move.cancel()
    yield util.sleep(5)
    yield sub.move.forward(.8).go()
    
    yield sub.move.up(5*.0254).go()
    yield sub.move.right(1.5*.0254).go()
    yield sub.fire_left_torpedo()
    yield sub.move.backward(2.5).go()

    yield sub.visual_approach('forward', 'shooter', size_estimate=5*.0254, desired_distance=1.5, selector=select_by_body_direction([0,-1,0]))
    yield util.sleep(5)
    yield sub.move.forward(.8).go()
    
    yield sub.move.up(5*.0254).go()
    yield sub.move.left(1.5*.0254).go()
    yield sub.fire_right_torpedo()
    yield sub.move.backward(2.5).go()
    """yield sub.move.right(.6).go()
    yield sub.move.forward(3.5).go()
    yield sub.move.left(1.2).go()
    yield sub.move.backward(3.5).go()
    yield sub.move.right(1.2).go()
    yield sub.move.forward(5).go()"""
