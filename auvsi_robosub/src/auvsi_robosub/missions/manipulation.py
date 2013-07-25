import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach import common_states, object_finder_states, missions
from object_finder.msg import TargetDesc
from uf_common.msg import PoseTwistStamped
from uf_common.orientation_helpers import PoseEditor
from geometry_msgs.msg import Quaternion

import numpy
import smach

BOARD_DIST = 3 # board-centering distance
WHEEL_DIST = 1 # wheel-searching distance
TURN_DIST = 0 # turning distance

def make_manipulation(shared):
    traj = PoseEditor.from_PoseTwistStamped_topic('/trajectory')

    manipulation_desc = TargetDesc()
    manipulation_desc.type = TargetDesc.TYPE_OBJECT
    manipulation_desc.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/manipulation.obj')
    manipulation_desc.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    manipulation_desc.disallow_yawing = True
    manipulation_desc.min_dist = BOARD_DIST-1
    manipulation_desc.max_dist = BOARD_DIST+3
    
    
    wheel = TargetDesc()
    wheel.type = TargetDesc.TYPE_OBJECT
    wheel.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/manipulation_wheel.obj')
    wheel.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    cov = numpy.zeros((6, 6))
    a = numpy.array([traj.forward_vector]).T * 100
    cov[3:, 3:] += a.dot(a.T)
    wheel.prior_distribution.covariance = cov.flatten()
    wheel.min_dist = WHEEL_DIST*.9
    wheel.max_dist = WHEEL_DIST/.9
    wheel.allow_rolling = True
    wheel.disallow_yawing = True

    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(1.5)))
        
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_MANIPULATION',
                           object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                    lambda: [manipulation_desc], .95))
        smach.Sequence.add('STOP',
                           common_states.VelocityState(shared, numpy.array([0, 0, 0])))
        smach.Sequence.add('SLEEP',
                           common_states.SleepState(5))
        smach.Sequence.add('APPROACH_MANIPULATION',
                           object_finder_states.ApproachObjectState(shared,
                                                                    'find_forward', 'forward_camera',
                                                                    BOARD_DIST))
        smach.Sequence.add('OPEN_LOOP_FORWARD',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.forward(BOARD_DIST-WHEEL_DIST).left(.5)))
        smach.Sequence.add('WAIT_WHEEL',
                           object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                    lambda: [wheel], .99))
        smach.Sequence.add('WAIT_WHEEL_MORE',
                           common_states.SleepState(5))
        smach.Sequence.add('APPROACH_WHEEL',
                           object_finder_states.ApproachObjectState(shared, 'find_forward',
                                                                    'forward_camera', WHEEL_DIST))
        smach.Sequence.add('EXTEND',
                           subjugator_states.GasPoweredStickState(True))
        smach.Sequence.add('OPEN_LOOP_FORWARD2',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.forward(WHEEL_DIST-TURN_DIST)\
                                                                      .relative([0, .06, .06])))
        smach.Sequence.add('TURN',
                           common_states.WaypointSeriesState(shared, [
                    lambda cur: cur.down(.2),
                    lambda cur: cur.right(.2),
                    lambda cur: cur.up(.2),
                    lambda cur: cur.left(.2),
                    lambda cur: cur.down(.2),
                    lambda cur: cur.right(.2)]))
        smach.Sequence.add('RETRACT',
                           subjugator_states.GasPoweredStickState(False))
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared, lambda cur: cur.backward(1)))
    return sm

missions.register_factory('manipulation', make_manipulation)
