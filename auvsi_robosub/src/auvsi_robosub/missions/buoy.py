import roslib; roslib.load_manifest('uf_smach')
from uf_smach.common_states import WaypointState, VelocityState
from uf_smach.object_finder_states import WaitForObjectsState, ApproachObjectState
from uf_smach import missions
from object_finder.msg import TargetDesc
from uf_smach.util import StateSharedHandles, left_position_selector, right_position_selector

import numpy
import rospy
import smach
import smach_ros

buoy_desc = TargetDesc()
buoy_desc.type = TargetDesc.TYPE_SPHERE
buoy_desc.sphere_radius = 4*.0254 # 4 in
buoy_desc.prior_distribution.pose.orientation.w = 1
#buoy_desc.prior_distribution.covariance[0+6*0] = 1
#buoy_desc.prior_distribution.covariance[1+6*1] = 1
#buoy_desc.prior_distribution.covariance[2+6*2] = 1
buoy_desc.min_dist = 4
buoy_desc.max_dist = 12

def make_buoy(shared):
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(1.5)))
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_BUOYS',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [buoy_desc]*3, .85),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('APPROACH_LEFT_BUOY',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', 2,
                                               left_position_selector))
        smach.Sequence.add('BUMP', WaypointState(shared, lambda cur: cur.forward(2.1)))
        smach.Sequence.add('BACKWARD', VelocityState(shared, numpy.array([-.2, 0, 0])))
        smach.Sequence.add('WAIT_BUOYS2',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [buoy_desc]*3, .85),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('APPROACH_RIGHT_BUOY',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', 2,
                                               right_position_selector))
        smach.Sequence.add('BUMP2', WaypointState(shared, lambda cur: cur.forward(2.1)))
        smach.Sequence.add('BACKWARD2', WaypointState(shared, lambda cur: cur.backward(1)))
        smach.Sequence.add('SURFACE', WaypointState(shared, lambda cur: cur.depth(.1)))

    return sm

missions.register_factory('buoy', make_buoy)
