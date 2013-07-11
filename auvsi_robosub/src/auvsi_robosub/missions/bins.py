import sys

import roslib; roslib.load_manifest('uf_smach')
from uf_smach.common_states import WaypointState, VelocityState, ServiceState
from uf_smach import legacy_vision_states, missions
from uf_smach.util import StateSharedHandles, left_orientation_selector, right_orientation_selector
from actuator_driver.srv import PulseValve

import numpy
import rospy
import smach
import smach_ros

DROP_ORDER = ['10', '16']

def unit_vector(x):
    return x/numpy.linalg.norm(x)

def select_image_text_or_most_central(image_text):
    def _(results, traj_start, (tf_p, tf_q)):
        if all('image_text' in result for result in results):
            matches = [result for result in results if result['image_text'] == image_text]
            assert len(matches) == 1 # guaranteed by BinsFinder
            return matches[0]
        else:
            return max(results, key=lambda result:
                unit_vector(numpy.array(map(float, result['center']))).dot([0, 0, 1])
            )
    return _

def make_bins(shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(.2)))
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.2, 0, 0])))
        
        for i, bin_string in enumerate(DROP_ORDER):
            if i != 0: # depth on first drop stops approach
                smach.Sequence.add(str(i)+'_DEPTH', WaypointState(shared, lambda cur: cur.depth(.2)))
            smach.Sequence.add(str(i)+'_WAIT_ALL', legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'bins/all'), transitions={'timeout': 'failed'})
            smach.Sequence.add(str(i)+'_CENTER_ALL', legacy_vision_states.CenterObjectState(shared, 'find2_down_camera'))
            smach.Sequence.add(str(i)+'_ALIGN_ALL', legacy_vision_states.AlignObjectState(shared, 'find2_down_camera', body_vec_align=[0, 1, 0]))
            smach.Sequence.add(str(i)+'_CENTER_ALL_TRY2', legacy_vision_states.CenterObjectState(shared, 'find2_down_camera'))
            
            # this could be faster if CenterObjectState let you descend at the same time
            smach.Sequence.add(str(i)+'_WAIT_SINGLE2', legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'bins/single'), transitions={'timeout': 'failed'})
            smach.Sequence.add(str(i)+'_CENTER_SINGLE2', legacy_vision_states.CenterObjectState(shared, 'find2_down_camera', selector=select_image_text_or_most_central(bin_string)))
            smach.Sequence.add(str(i)+'_DEPTH2', WaypointState(shared, lambda cur: cur.depth(1.5)))
            
            smach.Sequence.add(str(i)+'_WAIT_SINGLE3', legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'bins/single'), transitions={'timeout': 'failed'})
            smach.Sequence.add(str(i)+'_CENTER_SINGLE3', legacy_vision_states.CenterObjectState(shared, 'find2_down_camera', selector=select_image_text_or_most_central(bin_string)))
            smach.Sequence.add(str(i)+'_DEPTH3', WaypointState(shared, lambda cur: cur.depth(2.25)))
            
            smach.Sequence.add(str(i)+'_DROP', ServiceState('/actuator_driver/pulse_valve', PulseValve, 0, rospy.Duration(0.5)))

    return sm

missions.register_factory('bins', make_bins)
