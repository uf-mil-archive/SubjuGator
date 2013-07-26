import math

import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions
from uf_common.msg import PoseTwistStamped
from uf_common.orientation_helpers import PoseEditor
from geometry_msgs.msg import Quaternion

import numpy
import smach

class SaveZState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['z'])

    def execute(self, userdata):
        pose_editor = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        userdata.z = pose_editor.position[2]
        return 'succeeded'

class CompareZState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['higher', 'lower'], input_keys=['z'])
    
    def execute(self, userdata):
        pose_editor = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
        return 'higher' if pose_editor.position[2] > userdata.z else 'lower'

def make_manipulation(shared):
    sm_wheel = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')
    with sm_wheel:
        smach.Sequence.add('OPEN_LOOP_FORWARD',
                           common_states.WaypointState(shared,
                                       lambda cur: cur.forward(1.9), speed=.25))
        smach.Sequence.add('WAIT_WHEEL',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                    'grapes/grape'))
        smach.Sequence.add('APPROACH_WHEEL',
                           legacy_vision_states.CenterObjectState(shared, 'find2_forward_camera',
                                                    gain=.2))
        smach.Sequence.add('EXTEND',
                           subjugator_states.GasPoweredStickState(True))
        smach.Sequence.add('OPEN_LOOP_FORWARD2',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.forward(.85)\
                                                                      .relative([0, 0, 0])
                                                                      .relative([0, .075, .075]), speed=.1))
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
    
    sm_move_lever = smach.StateMachine(['succeeded', 'timeout', 'failed', 'preempted'], input_keys=['z'])
    with sm_move_lever:
        smach.StateMachine.add('COMPARE_Z', CompareZState(), transitions=dict(
            higher='GO_DOWN',
            lower='GO_UP',
        ))
        smach.StateMachine.add('GO_UP', common_states.WaypointSeriesState(shared, [
            lambda cur: cur.right(.08),
            lambda cur: cur.down(.15),
            lambda cur: cur.forward(1),
            lambda cur: cur.up(1),
        ]))
        smach.StateMachine.add('GO_DOWN', common_states.WaypointSeriesState(shared, [
            lambda cur: cur.right(.08),
            lambda cur: cur.up(.15),
            lambda cur: cur.forward(1),
            lambda cur: cur.down(1),
        ]))
    
    sm_lever = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')
    with sm_lever:
        smach.Sequence.add('OPEN_LOOP_FORWARD',
                           common_states.WaypointState(shared,
                                       lambda cur: cur.forward(1.9)))
        smach.Sequence.add('SAVE_Z', SaveZState())
        smach.Sequence.add('WAIT_LEVER',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                    'grapes/lever'))
        smach.Sequence.add('APPROACH_LEVER',
                           legacy_vision_states.CenterObjectState(shared, 'find2_forward_camera',
                                                    gain=.2))
        smach.Sequence.add('MOVE_LEVER', sm_move_lever)
    
    # Create a SMACH state machine
    sm = smach.Sequence(['succeeded', 'timeout', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(2.5)))
        
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.4, 0, 0])))
        smach.Sequence.add('WAIT_MANIPULATION',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                'grapes/board', timeout=15))
        smach.Sequence.add('APPROACH_MANIPULATION',
                           legacy_vision_states.CenterApproachObjectState(shared,
                                                            'find2_forward_camera',
                                                            desired_scale=130e3/3**2))
        smach.Sequence.add('YAW', common_states.WaypointState(shared,
            lambda cur: cur.heading(math.radians(-179 + (180 if cur.forward_vector[0] > 0 else 0)))))
        smach.Sequence.add('APPROACH_MANIPULATION_POST_YAW',
                           legacy_vision_states.CenterApproachObjectState(shared,
                                                            'find2_forward_camera',
                                                            desired_scale=130e3/3**2))
        smach.Sequence.add('WHEEL', sm_wheel, transitions=dict(failed='BACKUP'))
        
        smach.Sequence.add('BACKUP',
                           common_states.WaypointState(shared, lambda cur: cur.backward(3)))
        smach.Sequence.add('REFIND',
                           common_states.VelocityState(shared, numpy.array([.1, 0, 0])))
        smach.Sequence.add('WAIT_MANIPULATION2',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                'grapes/board'))
        smach.Sequence.add('APPROACH_MANIPULATION2',
                           legacy_vision_states.CenterApproachObjectState(shared,
                                                            'find2_forward_camera',
                                                            desired_scale=130e3/3**2))
        smach.Sequence.add('LEVER', sm_lever)
        smach.Sequence.add('BACKUP2',
                           common_states.WaypointState(shared, lambda cur: cur.backward(1)))
    return sm

missions.register_factory('manipulation_thresh', make_manipulation)
