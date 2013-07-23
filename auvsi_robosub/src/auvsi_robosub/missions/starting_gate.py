from auvsi_robosub import subjugator_states, constants
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

def make_starting_gate(shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('GO',
                           common_states.WaypointSeriesState(shared, [lambda cur: cur.depth(2),
                                                                      lambda cur: cur.forward(8)]))
        smach.Sequence.add('PIPE_DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(constants.PIPE_DEPTH)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared,
                                                       numpy.array([constants.PIPE_SPEED, 0, 0])))
        smach.Sequence.add('WAIT_PIPE',
                           legacy_vision_states.WaitForObjectsState(shared,
                                                                    'find2_down_camera', 'pipe'),
                           transitions={'timeout': 'failed'})
    return sm

missions.register_factory('starting_gate', make_starting_gate)
    
