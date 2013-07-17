from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

HEADING = 0

def make_starting_gate(shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('DEPTH_HEADING',
                           common_states.WaypointState(shared, lambda cur: cur.heading(0)
                                                                              .depth(1)))
        smach.Sequence.add('FORWARD',
                           common_states.WaypointState(shared, lambda cur: cur.forward(4)))
    return sm

missions.register_factory('starting_gate', make_starting_gate)
    
