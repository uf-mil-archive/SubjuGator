from __future__ import division

from legacy_vision import msg as legacy_vision_msg
import json
from txros import util
import numpy
import math
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
        obj = yield sub.visual_approach('forward', 'shooter/hole', size_estimate=5*.0254, desired_distance=1.5, selector=select_by_body_direction([0,1,0]))    
    finally:
        yield fwd_move.cancel()
    
    goal_mgr = sub._camera_2d_action_clients['forward'].send_goal(legacy_vision_msg.FindGoal(
        object_names=['shooter/board'],
    ))
    feedback = yield goal_mgr.get_feedback()
    res = map(json.loads, feedback.targetreses[0].object_results)  
    
    print 'about to align'  
    while True:
        print 'aligning'
        feedback = yield goal_mgr.get_feedback()
        res = map(json.loads, feedback.targetreses[0].object_results)
        if not res:
            continue
        angle = float(res[0]['orientation_error'])
        xdist = 1.5 * math.sin(angle/2)
        print angle, xdist
        yield sub.move.yaw_left(angle).go()
        yield sub.move.right(2*xdist).go()

        if abs(angle)<math.radians(5):
                break

    print 'done aligning'
    
    yield sub.visual_approach('forward', 'shooter/hole', size_estimate=5*.0254, desired_distance=1.0, selector=select_by_body_direction([0,1,0])) 
    yield util.sleep(5)
    yield sub.move.forward(.7).go()
    yield sub.move.up(5*.0254).go()
    yield sub.move.right(1.5*.0254).go()
    yield sub.fire_left_torpedo()
    yield sub.move.backward(2.5).go()
    
    print 'going to second hole'
    yield sub.visual_approach('forward', 'shooter/hole', size_estimate=5*.0254, desired_distance=1.0, selector=select_by_body_direction([0,-1,0]))
    yield util.sleep(5)
    yield sub.move.forward(0.7).go()
    
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
