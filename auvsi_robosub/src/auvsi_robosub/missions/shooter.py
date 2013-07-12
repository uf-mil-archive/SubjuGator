import roslib; roslib.load_manifest('uf_smach')
from uf_smach import missions
from uf_smach.common_states import WaypointState, VelocityState, ServiceState
from uf_smach.object_finder_states import WaitForObjectsState, ApproachObjectState
from object_finder.msg import TargetDesc
from uf_common.msg import PoseTwistStamped
from uf_common.orientation_helpers import PoseEditor
from geometry_msgs.msg import Quaternion
from uf_smach.util import StateSharedHandles, left_position_selector, right_position_selector
from actuator_driver.srv import PulseValve

import numpy
import rospy
import smach
import smach_ros

DIST1 = 2.5 # box-centering distance
DIST2 = 1 # hexagon-searching distance
DIST3 = 0.15 # shooting distance

HEXAGON = 'small'
#HEXAGON = 'large'
BOX = 'blue'

def make_shooter(shared):
    traj = PoseEditor.from_PoseTwistStamped_topic('/trajectory')

    shooter_desc = TargetDesc()
    shooter_desc.type = TargetDesc.TYPE_OBJECT
    shooter_desc.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/shooter.obj')
    shooter_desc.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    shooter_desc.disallow_yawing = True
    shooter_desc.min_dist = DIST1-1
    shooter_desc.max_dist = DIST1+3
    
    
    target = TargetDesc()
    target.type = TargetDesc.TYPE_OBJECT
    target.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/shooter_%sin_%s_hexagon.obj' % (7 if HEXAGON == 'small' else 12, BOX))
    target.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    cov = numpy.zeros((6, 6))
    a = numpy.array([traj.forward_vector]).T * 100
    cov[3:, 3:] += a.dot(a.T)
    target.prior_distribution.covariance = cov.flatten()
    target.min_dist = DIST2*.9
    target.max_dist = DIST2/.9
    target.allow_rolling = True
    target.disallow_yawing = True

    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH', WaypointState(shared, lambda cur: cur.depth(1.5)))
        
        smach.Sequence.add('APPROACH', VelocityState(shared, numpy.array([.4, 0, 0])))
        smach.Sequence.add('WAIT_SHOOTER',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [shooter_desc], .85))
        smach.Sequence.add('APPROACH_SHOOTER',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', DIST1, marker=BOX))
        
        smach.Sequence.add('OPEN_LOOP_FORWARD', WaypointState(shared, lambda cur: cur.forward(DIST1-DIST2)))
        smach.Sequence.add('WAIT_HEXAGON',
                           WaitForObjectsState(shared, 'find_forward',
                                                    [target], .85))
        smach.Sequence.add('APPROACH_HEXAGON',
                           ApproachObjectState(shared, 'find_forward',
                                               'forward_camera', DIST2))
        smach.Sequence.add('OPEN_LOOP_FORWARD2', WaypointState(shared, lambda cur: cur.forward(DIST2-DIST3).relative([0, .12, .18]))) # TODO get adjustments from sub
        smach.Sequence.add('SHOOT', ServiceState('/actuator_driver/pulse_valve', PulseValve, 3, rospy.Duration(0.3)))
        
    return sm
                           
missions.register_factory('shooter', make_shooter)
