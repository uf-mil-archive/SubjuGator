import roslib; roslib.load_manifest('uf_smach')
from uf_smach import common_states, hydrophone_states, missions
from uf_common import orientation_helpers

import smach
import functools

APPROACH_DEPTH = .5
GRAB_FREQ = 24.5
DROP_FREQ = 24.5

def make_hydrophones(freq, shared):
    sm_travel = smach.StateMachine(['succeeded', 'preempted'])
    with sm_travel:
        smach.StateMachine.add('HYDROPHONES',
                               hydrophone_states.HydrophoneTravelState(shared, freq),
                               transitions={'failed': 'GO_NNE'})
        smach.StateMachine.add('GO_NNE',
                               common_states.WaypointSeriesState(shared,
                                                                 [lambda cur: cur.set_orientation(orientation_helpers.NORTH).turn_right_deg(22.5),
                                                                  lambda cur: cur.forward(5)]),
                               transitions={'succeeded': 'HYDROPHONES'})    
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(APPROACH_DEPTH)))
        smach.Sequence.add('TRAVEL', sm_travel)
    return sm

def make_hydrophones_close(freq, shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(1.75)))
        smach.Sequence.add('HYDROPHONES_TRAVEL',
                           hydrophone_states.HydrophoneTravelState(shared, freq))
        smach.Sequence.add('HYDROPHONES_APPROACH',
                           hydrophone_states.HydrophoneApproachState(shared, freq))
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.backward(.3)))
    return sm

missions.register_factory('hydrophone_grab',
                          functools.partial(make_hydrophones, GRAB_FREQ*1e3))
missions.register_factory('hydrophone_drop',
                          functools.partial(make_hydrophones_close, DROP_FREQ*1e3))
