import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach.common_states import WaypointState, VelocityState, SleepState
from uf_smach import legacy_vision_states, missions
from uf_smach.util import StateSharedHandles, left_orientation_selector, right_orientation_selector

import numpy
import rospy
import smach
import smach_ros

def make_pizza(shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(.2)))
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_PIZZA',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'wreath'),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('CENTER_APPROACH_PIZZA',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_down_camera', desired_scale=200))
        smach.Sequence.add('ALIGN_PIZZA',
                           legacy_vision_states.AlignObjectState(shared, 'find2_down_camera'))
        smach.Sequence.add('OPEN_GRABBER',
                           subjugator_states.OpenGrabberState())
        smach.Sequence.add('DOWN', WaypointState(shared, lambda cur: cur.down(1.2).backward(.2)))
        smach.Sequence.add('WAIT', SleepState(2))
        smach.Sequence.add('CLOSE_GRABBER',
                           subjugator_states.CloseGrabberState(),
                           transitions={'empty': 'failed'})
    return sm

missions.register_factory('pizza', make_pizza) # Mmmm..
