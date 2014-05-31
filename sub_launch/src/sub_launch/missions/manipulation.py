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

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    sub.move.go(linear=[0.25, 0, 0])
    
    yield sub.visual_approach('forward', 'grapes/board', size_estimate=math.sqrt(2)*3*12*.0254, desired_distance=3)
    
    goal_mgr = sub._camera_2d_action_clients['forward'].send_goal(legacy_vision_msg.FindGoal(
        object_names=['grapes/empty_cell'],
    ))
    feedback = yield goal_mgr.get_feedback()
    res = map(json.loads, feedback.targetreses[0].object_results)
    
    res.sort(key=lambda x: float(x['redness']))
    empty = res[:4]
    filled = res[4:8]
    
    empty_coords = set()
    filled_coords = set()
    
    for obj in filled:
        x = float(obj['center'][0])
        y = float(obj['center'][1])
        if x < -.06:
            X = -1
        elif x > .06:
            X = 1
        else:
            X = 0
        if y < -.06:
            Y = -1
        elif y > .06:
            Y = 1
        else:
            Y = 0    
        filled_coords.add((X,Y))
        
    for obj in empty:
        x = float(obj['center'][0])
        y = float(obj['center'][1])
        if x < -.06:
            X = -1
        elif x > .06:
            X = 1
        else:
            X = 0
        if y < -.06:
            Y = -1
        elif y > .06:
            Y = 1
        else:
            Y = 0    
        empty_coords.add((X,Y))
    print empty_coords
    print filled_coords
    
    def gen_paths(unmoved_peg_coords, empty_coords):
        if not unmoved_peg_coords:
            yield []
            return
        for a in unmoved_peg_coords:
            for b in empty_coords:
                for rest in gen_paths(unmoved_peg_coords - {a}, (empty_coords | {a}) - {b}):
                    yield [(a, b)] + rest

    def dist((ax, ay), (bx, by)):
        return math.sqrt((ax-bx)**2 + (ay-by)**2)

    def cost(path):
        return sum(dist(a, b) for a, b in path) + \
            sum(dist(b, c) for (a, b), (c, d) in zip(path[:-1], path[1:]))

    plan=min(gen_paths(filled_coords, empty_coords), key=cost)
    
    storedpose=sub.move
    
    for a,b in plan:
        yield storedpose.relative([2,-a[0]*12*.0254, -a[1]*12*.0254]).go()
        yield storedpose.relative([2,-b[0]*12*.0254, -b[1]*12*.0254]).go()
        
        yield sub.visual_approach('forward', 'grapes/empty_cell', size_estimate=6*.0254, desired_distance=1)


