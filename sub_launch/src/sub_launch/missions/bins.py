from __future__ import division
from txros import util
import sub_scripting
#"Landing Site"
#the goal of this mission
#there are four bins. Bins 1 - 3 have options of two pictures, Bin 4 is always the same picture
#the robot carries two markers and one bin is marked primary and one bin is marked secondary
#the most points are awarded for dropping in primary and secondary, dropping in any gives partial

def selectPrimary(results, body_tf):
  print results
  for result in results:
    print result['image_text']
    if result['image_text'] == '02a':
      return result
  


@util.cancellableInlineCallbacks
def main(nh):
  sub = yield sub_scripting.get_sub(nh)

  yield sub.move.forward(3).go()

  yield sub.visual_align('down', 'bins/all', distance_estimate=1.5, desired_distance=2)
   
  #yield sub.move.up(1).go()
  
  yield sub.visual_align('down', 'bins/single', distance_estimate=1.5, selector=selectPrimary)
  
