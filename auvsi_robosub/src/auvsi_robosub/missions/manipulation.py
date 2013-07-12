import roslib; roslib.load_manifest('uf_smach')
from uf_smach.common_states import WaypointState, VelocityState, ServiceState, SleepState
from uf_smach import common_states, missions
from uf_smach.object_finder_states import WaitForObjectsState, ApproachObjectState
from object_finder.msg import TargetDesc
from uf_common.msg import PoseTwistStamped
from uf_common.orientation_helpers import PoseEditor
from geometry_msgs.msg import Quaternion
from uf_smach.util import StateSharedHandles, left_position_selector, right_position_selector
from actuator_driver.srv import SetValve

import numpy
import rospy
import smach
import smach_ros

DIST1 = 3 # board-centering distance
DIST2 = 1.25 # wheel-searching distance
DIST3 = 0 # turning distance

def make_manipulation(shared):
    traj = PoseEditor.from_PoseTwistStamped_topic('/trajectory')

    manipulation_desc = TargetDesc()
    manipulation_desc.type = TargetDesc.TYPE_OBJECT
    manipulation_desc.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/manipulation.obj')
    manipulation_desc.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    manipulation_desc.disallow_yawing = True
    manipulation_desc.min_dist = DIST1-1
    manipulation_desc.max_dist = DIST1+3
    
    
    wheel = TargetDesc()
    wheel.type = TargetDesc.TYPE_OBJECT
    wheel.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/manipulation_wheel.obj')
    wheel.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    cov = numpy.zeros((6, 6))
    a = numpy.array([traj.forward_vector]).T * 100
    cov[3:, 3:] += a.dot(a.T)
    wheel.prior_distribution.covariance = cov.flatten()
    wheel.min_dist = DIST2*.9
    wheel.max_dist = DIST2/.9
    wheel.allow_rolling = True
    wheel.disallow_yawing = True

    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(1.5)))
        
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.4, 0, 0])))
        smach.Sequence.add('WAIT_MANIPULATION',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [manipulation_desc], .85))
        smach.Sequence.add('APPROACH_MANIPULATION',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', DIST1))
        
        smach.Sequence.add('OPEN_LOOP_FORWARD', WaypointState(shared, lambda cur: cur.forward(DIST1-DIST2)))
        smach.Sequence.add('WAIT_WHEEL',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [wheel], .99))
        smach.Sequence.add('WAIT_WHEEL_MORE',
                           SleepState(5))
        smach.Sequence.add('APPROACH_WHEEL',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', DIST2))
        smach.Sequence.add('EXTEND',
                           ServiceState('/actuator_driver/set_valve', SetValve, 4, True))
        smach.Sequence.add('OPEN_LOOP_FORWARD2', WaypointState(shared, lambda cur: cur.forward(DIST2-DIST3).relative([0, .06, .06])))
        smach.Sequence.add('TURN',
                           common_states.WaypointSeriesState(shared, [
                    lambda cur: cur.down(.2),
                    lambda cur: cur.right(.2),
                    lambda cur: cur.up(.2),
                    lambda cur: cur.left(.2),
                    lambda cur: cur.down(.2),
                    lambda cur: cur.right(.2),
        ]))
        smach.Sequence.add('RETRACT',
                           ServiceState('/actuator_driver/set_valve', SetValve, 4, False))

        smach.Sequence.add('BACKUP',
                           WaypointState(shared, lambda cur: cur.backward(1)))
    return sm

missions.register_factory('manipulation', make_manipulation)
