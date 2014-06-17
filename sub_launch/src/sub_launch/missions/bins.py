from __future__ import division
from txros import util
import sub_scripting
import math
#"Landing Site"
#the goal of this mission
#there are four bins. Bins 1 - 3 have options of two pictures, Bin 4 is always the same picture
#the robot carries two markers and one bin is marked primary and one bin is marked secondary
#the most points are awarded for dropping in primary and secondary, dropping in any gives partial

def selectPrimary(results, body_tf):
  #print results
  for result in results:
    #print result['image_text']
    if 'image_text' in result and result['image_text'] == '02a':
      return result
    else:
      return select_centered(results, body_tf)
  
select_centered = lambda objs, body_tf: min(objs, key=lambda obj: math.sqrt(float(obj['center'][0])**2 + float(obj['center'][1])**2))

@util.cancellableInlineCallbacks
def main(nh):
  sub = yield sub_scripting.get_sub(nh)

  #yield sub.move.forward(1).go()
  print "aligning down"
  yield sub.visual_align('down', 'bins/all', distance_estimate=1.5)
  print "aligned down"   
  #yield sub.move.up(1).go()
  print "aligning single"
  yield sub.visual_align('down', 'bins/single', distance_estimate=1.5, selector=selectPrimary)
  
