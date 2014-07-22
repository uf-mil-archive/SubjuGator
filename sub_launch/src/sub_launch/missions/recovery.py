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
        yield sub.visual_align('down', 'wreath/cheese', dist, selector=select_by_body_direction([0,1,0]), turn=False)
    finally:
        yield fwd_move.cancel()       
    board_pose = sub.pose
    """while True:
        dist = yield sub.get_dvl_range()
        yield sub.move.down(dist-1.75).go()
        dist = yield sub.get_dvl_range()
        try:
            yield wrap_timeout(sub.visual_align('down', 'wreath/cheese', dist-.3, selector=select_by_body_direction([0,1,0]), turn=False), 20)
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
                yield wrap_timeout(sub.visual_align('down', 'wreath/cheese', dist-.3, selector=select_centered, turn=False), 20)
            except TimeoutError:
                print 'timed out'
                break
            print "relative move"
            yield sub.move.relative([-.07,-.1,0]).go()
            print "lower down grabber"
            yield sub.lower_down_grabber()
            print "opening down grabber"
            yield sub.open_down_grabber()
            print "moving down"
            yield sub.move.down(.95).go(speed=.15)
            print "close down grabber"
            yield sub.close_down_grabber()
            print "moving back to surface"
            yield sub.move.up(.5).go(speed=.1)
            yield sub.move.depth(.8).go()
            print "going to hydrophone"
            yield sub.hydrophone_align(25e3)
            print "opening down grabber"
            yield sub.open_down_grabber()
            yield util.sleep(1)
            yield "closing down grabber"
            yield sub.close_down_grabber()
            yield sub.raise_down_grabber()
            print "going back"
            yield sub.move.look_at_without_pitching(board_pose.position).go()
            yield sub.move.set_position(board_pose.position).go()
    finally:
        print "finally -> close -> raise -> orig depth"
        yield sub.close_down_grabber()
        yield sub.raise_down_grabber()
    yield sub.move.depth(orig_depth).go()
