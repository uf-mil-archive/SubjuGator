from __future__ import division
from sub_launch.missions import path
from txros import util

from twisted.internet import defer

import random
import time
import math
import sub_scripting
import numpy

RELATIVE_PINGER_MOVE = numpy.array([-.15, -.1, 0])
RELATIVE_VISION_MOVE = numpy.array([-.07, -.07, 0])

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
            
            if obj_name == 'cheese':
                return results[0]
            else:
                return results[-1]
            
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
def get_weight(sub):
    yield util.sleep(10)
    start = time.time()
    res = []
    while time.time() < start + 5:
        res.append((yield sub.get_z_force()))
    defer.returnValue(sum(res) / len(res))

@util.cancellableInlineCallbacks
def try_to_grab(sub, obj_name, freq, surface=False, bubbles=False):
    assert obj_name in ['moonrock', 'cheese']
    try:
        yield sub.move.depth(0.4).go()
        fwd_move = sub.move.go(linear=[0.25, 0, 0])
        try:
            yield sub.visual_align('down', 'wreath/board/high', 2, selector=select_centered, turn=True, angle=math.radians(45))
        finally:
            yield fwd_move.cancel()
        board_pose = sub.pose.depth(1).right(.6)
        
        yield sub.move.yaw_left_deg(90*random.randrange(4)).go()
        yield sub.move.depth(2).go()
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/moonrock/high', 2, selector=selector(obj_name), one_shot=True, turn=False), 20)
        except util.TimeoutError:
            print 'timed out'
            yield sub.move.yaw_left_deg(120).go()
            return
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/moonrock/high', 2, selector=select_centered), 20)
        except util.TimeoutError:
            print 'timed out 1'
            yield sub.move.yaw_left_deg(120).go()
            return
        #print "weighing"
        #w1 = yield get_weight(sub)
        #print 'w1', w1
        print "moving down"
        yield sub.move.down(1).go(speed=.2)
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/moonrock/low', 1, selector=select_centered, turn=False), 20)
        except util.TimeoutError:
            print 'timed out 2'
            return
        yield sub.lower_down_grabber()
        yield sub.open_down_grabber()
        yield sub.move.relative([-.10 if obj_name == 'moonrock' else -.115,-.15,0]).go()
        if obj_name == 'moonrock':
            yield sub.move.down(.73).go(speed=.2)
        else:
            yield sub.move.down(.81).go(speed=.2)
        yield sub.close_down_grabber()
        print "moving back to surface"
        #yield sub.move.up(.5).go(speed=.2)
        yield sub.move.depth(1).go()
        #w2 = yield get_weight(sub)
        #print 'w2', w2
        #gain = w2 - w1
        #print 'weight gained', gain
        #weights = {0: 0, 1: 2.97, 2: 4.00}
        #objects = min(weights, key=lambda w: abs(weights[w] - gain))
        #print 'object count', objects
        #if objects != 1:
        #    yield sub.move.set_position(board_pose.position + [random.uniform(-.5, .5), random.uniform(-.5, .5), 0]).go()
        #    yield sub.open_down_grabber()
        #    yield util.sleep(1)
        #    yield sub.close_down_grabber()
        #    yield sub.move.turn_left_deg(120).go()
        #    defer.returnValue(False)
        print "going to hydrophone"
        yield sub.hydrophone_align(freq)
        yield sub.move.relative(RELATIVE_PINGER_MOVE).go()
        yield sub.move.depth(.4).go()
        yield sub.raise_down_grabber()
        bin_pose = sub.move
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/bin/high', 2, selector=select_centered, turn=False), 10)
            yield sub.move.relative(RELATIVE_VISION_MOVE).go()
        except util.TimeoutError:
            print 'bin alignment timed out'
            yield bin_pose.go()
        except:
            print 'bin alignment???'
            yield bin_pose.go()
        if surface:
            yield sub.move.depth(0).go()
            yield sub.move.depth(.4).go()
        yield sub.lower_down_grabber()
        #yield sub.move.relative([-.15,-.2,0]).go()
        yield sub.move.depth(2).go()
        print "going down"
        yield sub.open_down_grabber()
        yield util.sleep(3)
        yield sub.close_down_grabber()
        yield util.sleep(1)
        yield sub.open_down_grabber()
        yield util.sleep(3)
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
    #yield sub.move.depth(2).go()
    defer.returnValue(True)

@util.cancellableInlineCallbacks
def retry_to_grab(*args, **kwargs):
    while True:
        res = yield try_to_grab(*args, **kwargs)
        if res: break

@util.cancellableInlineCallbacks
def main(nh, freq=25e3):
    sub = yield sub_scripting.get_sub(nh)
    yield sub.raise_down_grabber()

    if 1:
        yield sub.move.depth(1).go()
        fwd_move = sub.move.go(linear=[.3, 0, 0])
        try:
            yield sub.hydrophone_align(freq)
        finally:
            fwd_move.cancel()
        yield sub.move.relative(RELATIVE_PINGER_MOVE).go()
        
        #print 'surfacing'
        #yield sub.move.depth(0).go()
        
        yield sub.move.depth(0.4).go()
        bin_pose = sub.move
        try:
            yield util.wrap_timeout(sub.visual_align('down', 'wreath/bin/high', 2, selector=select_centered, turn=False), 10)
            yield sub.move.relative(RELATIVE_VISION_MOVE).go()
        except util.TimeoutError:
            print 'bin alignment timed out'
            yield bin_pose.go()
        except:
            print 'bin alignment???'
            yield bin_pose.go()
        yield path.main(nh, orient_away_from=True, forward=False, depth=0.4)
        yield sub.move.forward(2).go()
    
    yield retry_to_grab(sub, 'moonrock', freq, surface=True)
    yield retry_to_grab(sub, 'moonrock', freq, bubbles=True, surface=True)
    yield retry_to_grab(sub,   'cheese', freq)
    yield retry_to_grab(sub,   'cheese', freq)
    yield retry_to_grab(sub,   'cheese', freq, bubbles=True)
    yield retry_to_grab(sub, 'moonrock', freq)
    while True:
        yield retry_to_grab(sub, 'moonrock', freq)
