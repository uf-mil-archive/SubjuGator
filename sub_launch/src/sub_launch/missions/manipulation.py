#blue cell dia 6in * .0254 
#peg dia 4in * .0254
#yield sub.visual_approach('forward', 'peg', size_estimate=6*.0254, desired_distance=2)
#yield sub.visual_approach('forward', 'cell', size_estimate=4*.0254, desired_distance=2)
from __future__ import division
import math
from txros import util
from legacy_vision import msg as legacy_vision_msg
import sub_scripting
import json
import itertools
from scipy.stats.stats import pearsonr

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    fwd_move = sub.move.go(linear=[0.25, 0, 0])
    try:
        yield sub.visual_approach('forward', 'grapes/board', size_estimate=math.sqrt(2)*3*12*.0254, desired_distance=2)
    finally:
        yield fwd_move.cancel()
    """
    goal_mgr = sub._camera_2d_action_clients['forward'].send_goal(legacy_vision_msg.FindGoal(
        object_names=['grapes/board2'],
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
        #goal_mgr.cancel()
        angle = float(res[0]['orientation_error'])
        xdist = 1.5 * math.sin(angle/2)
        print angle, xdist
        yield sub.move.yaw_left(angle).go()
        yield sub.move.right(2*xdist).go()

        if abs(angle)<math.radians(5):
                break

    print 'done aligning'
    """
    goal_mgr = sub._camera_2d_action_clients['forward'].send_goal(legacy_vision_msg.FindGoal(
        object_names=['grapes/empty_cell'],
    ))
    feedback = yield goal_mgr.get_feedback()
    res = map(json.loads, feedback.targetreses[0].object_results)
    
    res.sort(key=lambda x: float(x['redness']))
    
    all_possible_coords=set((X,Y) for X in [-1, 0, 1] for Y in [-1, 0, 1] if X != 0 or Y != 0)
    coords = max(itertools.permutations(all_possible_coords, len(res)),
        key=lambda positions: min(
            pearsonr(*zip(*((pos[0], float(obj['center'][0])) for pos, obj in zip(positions, res)))),
            pearsonr(*zip(*((pos[1], float(obj['center'][1])) for pos, obj in zip(positions, res)))),
        )
    )
        
    empty = res[:4]
    filled = res[4:8]
    
    for x in res:
        print x['redness'], x['center']

    
    '''empty_coords = min(itertools.permutations(all_possible_coords, 4),
        key=lambda positions: sum(math.sqrt(
            (float(obj['center'][0]) - .12*pos[0])**2 +
            (float(obj['center'][1]) - .12*pos[1])**2
        ) for pos, obj in zip(positions, empty)),
    )'''
    empty_coords = coords[:4]
    print empty_coords
    
    empty_coords = set(empty_coords)
    
    filled_coords = all_possible_coords - empty_coords
    print empty_coords, filled_coords
    
    def gen_paths(unmoved_peg_coords, empty_coords):
        if not unmoved_peg_coords:
            yield []
            return
        for a in unmoved_peg_coords:
            for b in empty_coords:
                for rest in gen_paths(unmoved_peg_coords - {a}, empty_coords - {b}):
                    yield [(a, b)] + rest

    def dist((ax, ay), (bx, by)):
        return math.sqrt((ax-bx)**2 + (ay-by)**2)

    def cost(path):
        return sum(dist(a, b) for a, b in path) + \
            sum(dist(b, c) for (a, b), (c, d) in zip(path[:-1], path[1:]))

    plan=min(gen_paths(filled_coords, empty_coords), key=cost)
    
    storedpose=sub.move
    
    print plan
    
    select_centered = lambda objs, body_tf: min(objs, key=lambda obj: math.sqrt(float(obj['center'][0])**2 + float(obj['center'][1])**2))
    
    for a,b in plan:
        print 'going to', a
        #yield storedpose.relative([0,-a[0]*12*.0254, -a[1]*12*.0254]).go(speed=.1)
        
        yield storedpose.relative([1,-float(res[coords.index(a)]['center'][0])*1.5, -float(res[coords.index(a)]['center'][1])*1.5]).go(speed=.1)
        #center on peg
        
        yield sub.open_gripper()
        
        yield sub.set_ignore_magnetometer(True)
        try:
            yield sub.visual_approach('forward', 'grapes/peg', size_estimate=6*.0254, desired_distance=1, selector=select_centered)
            yield sub.move.right(.125).go()
            yield sub.move.down(.03).go()
            yield sub.move.forward(.32).go()
            print 'gripper closing'
            yield sub.close_gripper()
            yield sub.move.backward(.75).go()
        finally:
            yield sub.set_ignore_magnetometer(False)
            
        print 'going to', b
        yield storedpose.relative([1,-float(res[coords.index(b)]['center'][0])*1.5, -float(res[coords.index(b)]['center'][1])*1.5]).go(speed=.1)
        print 'centering on', b
        
        yield sub.set_ignore_magnetometer(True) 
        try:       
            yield sub.visual_approach('forward', 'grapes/empty_cell', size_estimate=6*.0254, desired_distance=1, selector=select_centered)
            yield sub.move.right(.09).go()
            yield sub.move.down(.03).go()
            yield sub.move.forward(.54).go()
            print 'gripper opening'
            yield sub.open_gripper()
            yield sub.move.backward(.75).go()
        finally:
            yield sub.set_ignore_magnetometer(False)

