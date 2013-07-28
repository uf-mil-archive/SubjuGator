import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import constants
from uf_smach import common_states, hydrophone_states, missions
from uf_common import orientation_helpers

import smach
import functools

APPROACH_DEPTH = .5
FREQ = 24.5e3 if constants.MODE == 'competition' else 27e3
FREQ_RANGE = 1.25e3

def make_hydrophones(shared):
    sm_travel = smach.StateMachine(['succeeded', 'preempted'])
    with sm_travel:
        smach.StateMachine.add('HYDROPHONES',
                               hydrophone_states.HydrophoneTravelState(shared, FREQ, FREQ_RANGE),
                               transitions={'failed': 'GO_AWAY'})
        if constants.MODE == 'competition':
            smach.StateMachine.add('GO_AWAY',
                                   common_states.WaypointSeriesState(shared,
                                                                     [lambda cur: cur.set_orientation(orientation_helpers.NORTH).turn_right_deg(22.5),
                                                                      lambda cur: cur.forward(5)]),
                                   transitions={'succeeded': 'HYDROPHONES'})
        else:
            smach.StateMachine.add('GO_AWAY',
                                   common_states.WaypointSeriesState(shared,
                                                                     [lambda cur: cur.set_orientation(orientation_helpers.NORTH).turn_left_deg(22.5),
                                                                      lambda cur: cur.forward(5)]),
                                   transitions={'succeeded': 'HYDROPHONES'})    

    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(APPROACH_DEPTH)))
        smach.Sequence.add('TRAVEL', sm_travel)
    return sm

def make_hydrophones_close(shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(1.5)))
        smach.Sequence.add('HYDROPHONES_TRAVEL',
                           hydrophone_states.HydrophoneTravelState(shared, FREQ, FREQ_RANGE),
                           transitions={'failed': 'HYDROPHONES_TRAVEL'})
        smach.Sequence.add('HYDROPHONES_APPROACH',
                           hydrophone_states.HydrophoneApproachState(shared, FREQ, FREQ_RANGE),
                           transitions={'failed': 'succeeded'}) # Ensure we drop
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.backward(.3)))
    return sm

missions.register_factory('hydrophone_grab', make_hydrophones)
missions.register_factory('hydrophone_drop', make_hydrophones_close)
