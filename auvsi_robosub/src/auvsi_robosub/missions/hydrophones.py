import roslib; roslib.load_manifest('uf_smach')
from uf_smach import common_states, hydrophone_states, missions

import smach
import functools

def make_hydrophones(freq, shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        # Add states to the container
        smach.Sequence.add('DEPTH', common_states.WaypointState(shared, lambda cur: cur.depth(.5)))
        smach.Sequence.add('HYDROPHONES', hydrophone_states.HydrophoneTravelState(shared, freq))
    return sm

for freq in [23, 25, 27]:
    missions.register_factory('hydrophone_%dkhz' % freq,
                              functools.partial(make_hydrophones, freq*1e3))
