from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

def make_hedge(shared):
    sm_approach = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_approach:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(1)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_HEDGE',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_forward_camera',
                                                                    'hedge',
                                                                    timeout=15),
                           transitions={'timeout': 'failed'})
    
    sm_center = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_center:
        smach.Sequence.add('CENTER_APPROACH_HEDGE',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_forward_camera',
                                                                          desired_scale=250))
        smach.Sequence.add('GO',
                           common_states.WaypointState(shared, lambda cur: cur.forward(4)))
    
    sm_pipe = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_pipe:
        smach.Sequence.add('DEPTH', common_states.WaypointState(shared,
                                                                lambda cur: cur.depth(.3)))

        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared,
                                                       numpy.array([.3, 0, 0])))
        smach.Sequence.add('WAIT_PIPE',
                           legacy_vision_states.WaitForObjectsState(shared,
                                                                    'find2_down_camera', 'pipe'),
                           transitions={'timeout': 'failed'})

    sm = smach.StateMachine(['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('APPROACH', sm_approach,
                               transitions={'succeeded': 'CENTER',
                                            'failed': 'PIPE'})
        smach.StateMachine.add('CENTER', sm_approach,
                               transitions={'succeeded': 'PIPE',
                                            'failed': 'PIPE'})
        smach.StateMachine.add('PIPE', sm_pipe,
                               transitions={'succeeded': 'succeeded'})
    return sm

missions.register_factory('hedge', make_hedge)
