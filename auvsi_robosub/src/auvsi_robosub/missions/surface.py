from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

def make_surface(shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('SURFACE',
                           common_states.WaypointState(shared, lambda cur: cur.depth(.05)))
        smach.Sequence.add('SLEEP',
                           common_states.SleepState(5))
        smach.Sequence.add('DOWN',
                           common_states.WaypointState(shared, lambda cur: cur.depth(.3)))
    return sm

missions.register_factory('surface', make_surface)
