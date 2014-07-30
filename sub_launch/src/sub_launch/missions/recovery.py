from __future__ import division
from sub_launch.missions import path
from txros import util

import math
import sub_scripting
import numpy

def selector(obj_name):
    assert obj_name in ['moonrock', 'cheese']
    def _(results, body_tf):
        results = list(results)
        if len(results) > 1:
            for obj in results: obj['redness'] = float(obj['redness'])
            results.sort(key=lambda obj: float(obj['redness']))
            def ene(x):
                x = numpy.array(x)
                y = x - numpy.mean(x)
                return numpy.sum(y*y)
            first_rock = min(xrange(1, len(results)), key=lambda i: ene([x['redness'] for x in results[:i]]) + ene([x['redness'] for x in results[i:]]))
            cheeses, rocks = results[:first_rock], results[first_rock:]
            print [c['redness'] for c in cheeses], [r['redness'] for r in rocks]
            
            objs = cheeses if obj_name == 'cheese' else rocks
            
            return min(objs, key=lambda obj: math.sqrt(float(obj['center'][0])**2 + float(obj['center'][1])**2))
        else:
            return results[0]
        
    return _
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
def try_to_grab(sub, obj_name, board_pose, surface=False, bubbles=False):
    assert obj_name in ['moonrock', 'cheese']
    try:
        yield sub.move.depth(2).go()
        dist = yield sub.get_dvl_range()
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/moonrock/high', 2, selector=selector(obj_name)), 20)
        except util.TimeoutError:
            print 'timed out'
            return
        print "relative move"
        yield sub.lower_down_grabber()
        yield sub.open_down_grabber()
        yield util.sleep(5) # XXX
        print "moving down"
        yield sub.move.down(1).go(speed=.2)
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/moonrock/low', 2, selector=select_centered, turn=False), 20)
        except util.TimeoutError:
            print 'timed out'
            return
        yield sub.move.relative([-.09,-.15,0]).go()
        yield sub.move.down(.8).go(speed=.2)
        yield sub.close_down_grabber()
        print "moving back to surface"
        #yield sub.move.up(.5).go(speed=.2)
        yield sub.move.depth(2).go()
        yield util.sleep(5) # XXX
        print "going to hydrophone"
        yield sub.hydrophone_align(25e3)
        if surface:
            yield sub.move.depth(-.5).go()
            yield sub.move.depth(2).go()
        #yield sub.move.relative([-.15,-.2,0]).go()
        print "going down"
        yield sub.open_down_grabber()
        yield util.sleep(1)
        yield sub.close_down_grabber()
        yield sub.raise_down_grabber()
        if bubbles:
            yield sub.fire_left_torpedo()
            yield sub.fire_right_torpedo()
            yield sub.fire_left_torpedo()
            yield sub.fire_right_torpedo()
            yield sub.fire_left_torpedo()
            yield sub.fire_right_torpedo()
        print "going to board"
        yield sub.move.look_at_without_pitching(board_pose.position).go()
        yield sub.move.set_position(board_pose.position).go()
    finally:
        print "finally"
        yield sub.close_down_grabber()
        yield sub.raise_down_grabber()
    yield sub.move.depth(2).go() 

@util.cancellableInlineCallbacks
def main(nh, freq=25e3):
    sub = yield sub_scripting.get_sub(nh)
    yield sub.move.depth(1).go()
    yield sub.hydrophone_align(freq)
    
    print 'surfacing'
    yield sub.move.depth(-.5).go()
    yield sub.move.depth(1).go()
    
    yield path.main(nh, orient_away_from=True)
    orig_depth = -sub.pose.position[2]
    dist = yield sub.get_dvl_range()
    yield sub.move.depth(2).go()
    yield sub.move.forward(3).go()
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        yield sub.visual_align('down', 'wreath/board/high', 2, selector=select_centered, turn=True, angle=math.radians(45))
        #yield sub.visual_align('down', 'wreath/moonrock', dist, selector=select_by_body_direction([0,1,0]), turn=False)
    finally:
        yield fwd_move.cancel()
    board_pose = sub.pose
    
    yield try_to_grab(sub, 'moonrock', board_pose, surface=True)
    yield try_to_grab(sub, 'moonrock', board_pose)
    yield try_to_grab(sub, 'moonrock', board_pose, bubbles=True)
    yield try_to_grab(sub, 'cheese', board_pose)
    yield try_to_grab(sub, 'cheese', board_pose)
    yield try_to_grab(sub, 'cheese', board_pose)
    
    yield sub.move.depth(2).go()
