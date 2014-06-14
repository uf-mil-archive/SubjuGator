from __future__ import division
from txros import util
import sub_scripting
#"Landing Site"
#the goal of this mission
#there are four bins. Bins 1 - 3 have options of two pictures, Bin 4 is always the same picture
#the robot carries two markers and one bin is marked primary and one bin is marked secondary
#the most points are awarded for dropping in primary and secondary, dropping in any gives partial

@util.cancellableInlineCallbacks
def main(nh):
  sub = yield sub_scripting.get_sub(nh)
  sub.move.forward(3)
