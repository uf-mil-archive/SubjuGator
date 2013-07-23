import roslib; roslib.load_manifest('uf_smach')
from uf_smach import common_states, hydrophone_states, missions

import smach
import functools

APPROACH_DEPTH = .5
GRAB_FREQ = 25
DROP_FREQ = 26

def make_hydrophones(freq, shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(APPROACH_DEPTH)))
        smach.Sequence.add('HYDROPHONES',
                           hydrophone_states.HydrophoneTravelState(shared, freq))
    return sm

def make_hydrophones_close(freq, shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(APPROACH_DEPTH)))
        smach.Sequence.add('HYDROPHONES_TRAVEL',
                           hydrophone_states.HydrophoneTravelState(shared, freq))
        smach.Sequence.add('HYDROPHONES_APPROACH',
                           hydrophone_states.HydrophoneApproachState(shared, freq))
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.backward(.5)))
    return sm

missions.register_factory('hydrophone_grab',
                          functools.partial(make_hydrophones, GRAB_FREQ*1e3))
missions.register_factory('hydrophone_drop',
                          functools.partial(make_hydrophones_close, DROP_FREQ*1e3))
