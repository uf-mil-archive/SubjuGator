import roslib; roslib.load_manifest('uf_smach')
from uf_smach.common_states import WaypointState
from uf_smach.hydrophone_states import HydrophoneTravelState
from uf_smach.util import StateSharedHandles
from uf_smach import missions

import rospy
import smach
import smach_ros
import functools

def make_hydrophones(freq, shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        # Add states to the container
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(.5)))
        smach.Sequence.add('HYDROPHONES', HydrophoneTravelState(shared, freq))
    return sm

for freq in [23, 25, 27]:
    missions.register_factory('hydrophone_%dkhz' % freq,
                              functools.partial(make_hydrophones, freq*1e3))
