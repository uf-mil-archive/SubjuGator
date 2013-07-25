import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach import missions, common_states, legacy_vision_states

import numpy
import smach

DEPTH = 3

BOARD_SCALE = 2000
HEXAGON_SCALE = 3000
ALIGN_FORWARD = .85
ALIGN_STRAFE = 0.10
ALIGN_UP = 0.12

SIZE = 'small'
COLORS = ['red', 'yellow']
    
def make_shooter(shared):
    # Create a SMACH state machine
    sm_approach = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_approach:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(DEPTH)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.1, 0, 0])))
    sm_shoots = []
    for color, shooter in zip(COLORS, ['left', 'right']):
        sm_shoot = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
        sm_shoots.append(sm_shoot)
        with sm_shoot:
            # smach.Sequence.add('WAIT_SHOOTER',
            #                    legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
            #                                                             '/'.join(['shooter', color, 'box'])),
            #                    transitions={'timeout': 'failed'})
            # smach.Sequence.add('APPROACH_SHOOTER',
            #                    legacy_vision_states.CenterApproachObjectState(shared, 'find2_forward_camera',
            #                                                                   desired_scale=BOARD_SCALE))
            smach.Sequence.add('WAIT_HEXAGON',
                               legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                        '/'.join(['shooter_flood', color, SIZE])),
                               transitions={'timeout': 'failed'})
            smach.Sequence.add('APPROACH_HEXAGON',
                               legacy_vision_states.CenterApproachObjectState(shared, 'find2_forward_camera',
                                                                              desired_scale=HEXAGON_SCALE,
                                                                              gain=.5))
            if shooter == 'left':
                smach.Sequence.add('OPEN_LOOP_FORWARD',
                                   common_states.WaypointState(shared,
                                                               lambda cur: cur.forward(ALIGN_FORWARD)\
                                                                              .right(ALIGN_STRAFE)\
                                                                              .up(ALIGN_UP)))
            else:
                smach.Sequence.add('OPEN_LOOP_FORWARD',
                                   common_states.WaypointState(shared,
                                                               lambda cur: cur.forward(ALIGN_FORWARD)\
                                                                              .left(ALIGN_STRAFE)\
                                                                              .up(ALIGN_UP)))
            smach.Sequence.add('SLEEP', common_states.SleepState(3))
            smach.Sequence.add('SHOOT', subjugator_states.ShootTorpedoState(shooter))
            smach.Sequence.add('SLEEP2', common_states.SleepState(3))

    sm_retreat = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_retreat:
        smach.Sequence.add('RETREAT',
                           common_states.VelocityState(shared, numpy.array([-.2, 0, 0])))
        smach.Sequence.add('WAIT',
                           common_states.SleepState(4))
        
    sm = smach.StateMachine(['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('APPROACH_SHOOT_1', sm_approach,
                               transitions={'succeeded': 'SHOOT_1'})
        smach.StateMachine.add('RETREAT_SHOOT_1', sm_retreat,
                               transitions={'succeeded': 'SHOOT_1'})
        smach.StateMachine.add('SHOOT_1', sm_shoots[0],
                               transitions={'succeeded': 'RETREAT_SHOOT_2',
                                            'failed': 'RETREAT_SHOOT_1'})
        smach.StateMachine.add('RETREAT_SHOOT_2', sm_retreat,
                               transitions={'succeeded': 'SHOOT_2'})
        smach.StateMachine.add('SHOOT_2', sm_shoots[1],
                               transitions={'succeeded': 'DONE_BACKUP',
                                            'failed': 'RETREAT_SHOOT_2'})
        smach.StateMachine.add('DONE_BACKUP',
                               common_states.WaypointState(shared,
                                                           lambda cur: cur.backward(.5)),
                               transitions={'succeeded': 'succeeded'})
    return sm

def make_shooter_approach(shared):
    sm = common_states.WaypointSeriesState(shared,
                                           [lambda cur: cur.turn_left_deg(55)])
    return sm

missions.register_factory('shooter', make_shooter)
missions.register_factory('shooter_approach', make_shooter_approach)
