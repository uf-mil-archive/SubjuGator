import numpy
import math
from txros import util

import sub_scripting

@util.cancellableInlineCallbacks
def main(nh, sub=None):
    print "Q"
    if sub is None:
        sub = yield sub_scripting.get_sub(nh)
    
    while True:
        
        angles = yield sub.get_red_marker()
        print "Marker at", angles
        if abs(angles.x)>10 or abs(angles.y)>10:
            if angles.x<0:
                yield sub.move.left(abs(angles.x/500)).go()
            if angles.x>0:
                yield sub.move.right(abs(angles.x/500)).go()
            if angles.y>0:
                yield sub.move.down(abs(angles.y/500)).go()
            if angles.y<0:
                yield sub.move.up(abs(angles.y/500)).go()
        else:
            #yield sub.move.forward(1)).go()
            break

    print 'CENTERED ON MARKER'
    yield sub.move.forward(2).go()

    while True:
        
        angles = yield sub.get_red_marker()
        print "Marker at", angles
        if abs(angles.x)>10 or abs(angles.y)>10:
            if angles.x<0:
                yield sub.move.left(abs(angles.x/500)).go()
            if angles.x>0:
                yield sub.move.right(abs(angles.x/500)).go()
            if angles.y>0:
                yield sub.move.down(abs(angles.y/500)).go()
            if angles.y<0:
                yield sub.move.up(abs(angles.y/500)).go()
        else:
            #yield sub.move.forward(1)).go()
            break

    print 'CENTERED ON MARKER'

    yield sub.move.up(0.7).go()
    #yield sub.move.forward(1).go()
    yield sub.move.turn_left_deg(90).go()
    yield sub.move.right(3).go()
    yield sub.move.turn_left_deg(-90).go()





        

        
