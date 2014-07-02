from __future__ import division
from txros import util
import sub_scripting
import math
#"Landing Site"
#the goal of this mission
#there are four bins. Bins 1 - 3 have options of two pictures, Bin 4 is always the same picture
#the robot carries two markers and one bin is marked primary and one bin is marked secondary
#the most points are awarded for dropping in primary and secondary, dropping in any other gives partial
  
select_centered = lambda objs, body_tf: min(objs, key=lambda obj: math.sqrt(float(obj['center'][0])**2 + float(obj['center'][1])**2))
primary_secondary = ['2', '4']

def select(image_text):
    def _(results, body_tf):
        #print results
        for result in results:
            if 'image_text' in result and result['image_text'].startswith(image_text):
   		print result
		return result
        return None
        #return select_centered(results, body_tf)
    return _

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    orig_depth = -sub.pose.position[2]

    #yield sub.move.forward(1).go()
    print "aligning down"
    dist = yield sub.get_dvl_range()
    yield sub.visual_align('down', 'bins/all', distance_estimate=dist-.3, turn=True)
    yield sub.move.turn_left_deg(45).go()
    yield sub.visual_align('down', 'bins/all', distance_estimate=dist-.3, turn=False)
    #yield sub.move.down(dist-.3 - 2.5).go()
    print "aligned down"
    centered = sub.move
    
    for x in ["2", "4"]:
        print 'going to', x
        yield centered.go()
        print "aligning single", x
        dist = yield sub.get_dvl_range()
        yield sub.visual_align('down', 'bins/single', distance_estimate=dist-.3, selector=select(x), turn=False)
        print 'done'
        yield sub.move.down(dist - 1).go()
	yield sub.move.backward(.25).go()
        print 'dropping'
        yield sub.drop_ball()
        yield util.sleep(5)
        print 'done dropping'
    
    yield sub.move.depth(orig_depth).go()
