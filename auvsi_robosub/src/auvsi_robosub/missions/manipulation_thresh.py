import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions
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

    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')

    # Open the container
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(3)))
        
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.4, 0, 0])))
        smach.Sequence.add('WAIT_MANIPULATION',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                'grapes/board'))
        smach.Sequence.add('APPROACH_MANIPULATION',
                           legacy_vision_states.CenterApproachObjectState(shared,
                                                            'find2_forward_camera',
                                                            desired_scale=130e3/BOARD_DIST**2))
        smach.Sequence.add('OPEN_LOOP_FORWARD',
                           common_states.WaypointState(shared,
                                       lambda cur: cur.forward(BOARD_DIST-WHEEL_DIST).left(.25)))
        smach.Sequence.add('WAIT_WHEEL',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                    'grapes/grape'))
        smach.Sequence.add('APPROACH_WHEEL',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_forward_camera',
                                                    desired_scale=30e3/WHEEL_DIST**2, gain=.2))
        smach.Sequence.add('EXTEND',
                           subjugator_states.GasPoweredStickState(True))
        smach.Sequence.add('OPEN_LOOP_FORWARD2',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.forward(WHEEL_DIST-TURN_DIST)\
                                                                      .relative([0, .06, 0])))
        smach.Sequence.add('TURN',
                           common_states.WaypointSeriesState(shared, [
                    lambda cur: cur.down(.25),
                    lambda cur: cur.right(.25),
                    lambda cur: cur.up(.25),
                    lambda cur: cur.left(.25),
                    lambda cur: cur.down(.25),
                    lambda cur: cur.right(.25)]))
        smach.Sequence.add('RETRACT',
                           subjugator_states.GasPoweredStickState(False))
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared, lambda cur: cur.backward(1)))
    return sm

missions.register_factory('manipulation_thresh', make_manipulation)
