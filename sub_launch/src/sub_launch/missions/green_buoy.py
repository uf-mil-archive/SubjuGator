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
        
        angles = yield sub.get_green_buoy()
        print "Buoy at", angles
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
    

    print 'CENTERED ON BUOY'
    yield sub.move.forward(3.5).go()

    while True:
        
        angles = yield sub.get_green_buoy()
        print "Buoy at", angles
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
            
    print 'CENTERED ON BUOY'
    yield sub.move.forward(3.5).go()
    yield sub.move.forward(-6).go()




        

        
