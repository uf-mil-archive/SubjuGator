import roslib; roslib.load_manifest('uf_smach')
from uf_smach.common_states import WaypointState, VelocityState, ServiceState
from uf_smach import legacy_vision_states, missions
from uf_smach.util import StateSharedHandles, left_orientation_selector, right_orientation_selector
from actuator_driver.srv import SetValve

import numpy
import rospy
import smach
import smach_ros

def make_pizza(shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(.4)))
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_PIZZA',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'wreath'),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('CENTER_APPROACH_PIZZA',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_down_camera', desired_scale=90))
        smach.Sequence.add('ALIGN_PIZZA',
                           legacy_vision_states.AlignObjectState(shared, 'find2_down_camera'))
        smach.Sequence.add('OPEN',
                           ServiceState('/actuator_driver/set_valve', SetValve, 2, False))
        smach.Sequence.add('OPEN2',
                           ServiceState('/actuator_driver/set_valve', SetValve, 1, True))
        smach.Sequence.add('DOWN', WaypointState(shared, lambda cur: cur.down(.5)))
        smach.Sequence.add('CLOSE',
                           ServiceState('/actuator_driver/set_valve', SetValve, 1, False))
        smach.Sequence.add('CLOSE2',
                           ServiceState('/actuator_driver/set_valve', SetValve, 2, True))
        smach.Sequence.add('UP', WaypointState(shared, lambda cur: cur.up(2)))
    return sm

missions.register_factory('pizza', make_pizza) # Mmmm..
