from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

APPROACH_DEPTH = .3

def make_grabber(shared):
    sm_approach = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_approach:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(APPROACH_DEPTH)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_PIZZA',
                           legacy_vision_states.WaitForObjectsState(shared, 'find2_down_camera', 'wreath',
                                                                    timeout=9999),
                           transitions={'timeout': 'failed'})
        
    sm_descend_grab = smach.Sequence(['succeeded', 'failed', 'empty', 'preempted'], 'succeeded')
    with sm_descend_grab:
        smach.Sequence.add('CENTER_APPROACH_PIZZA',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_down_camera',
                                                                          desired_scale=90, gain=1))
        smach.Sequence.add('ALIGN_PIZZA',
                           legacy_vision_states.AlignObjectState(shared, 'find2_down_camera'))
        smach.Sequence.add('CENTER_APPROACH_PIZZA2',
                           legacy_vision_states.CenterApproachObjectState(shared, 'find2_down_camera',
                                                                          desired_scale=120, gain=.5))
        smach.Sequence.add('OPEN_GRABBER',
                           subjugator_states.OpenGrabberState())
        smach.Sequence.add('DOWN',
                           common_states.WaypointState(shared, lambda cur: cur.down(1.1)))
        smach.Sequence.add('WAIT',
                           common_states.SleepState(1))
        smach.Sequence.add('CLOSE_GRABBER',
                           subjugator_states.CloseGrabberState())

    sm_extra_grab = smach.Sequence(['succeeded', 'empty', 'preempted'], 'succeeded')
    with sm_extra_grab:
        smach.Sequence.add('OPEN_GRABBER',
                           subjugator_states.OpenGrabberState())
        smach.Sequence.add('DOWN',
                           common_states.WaypointState(shared, lambda cur: cur.down(.1)))
        smach.Sequence.add('WAIT',
                           common_states.SleepState(1))
        smach.Sequence.add('CLOSE_GRABBER',
                           subjugator_states.CloseGrabberState())
                           
    sm = smach.StateMachine(['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('APPROACH', sm_approach,
                               transitions={'succeeded': 'DESCEND_GRAB'})
        smach.StateMachine.add('DESCEND_GRAB', sm_descend_grab,
                               transitions={'succeeded': 'UP',
                                            'failed': 'APPROACH',
                                            'empty': 'EXTRA_GRAB'})
        smach.StateMachine.add('EXTRA_GRAB', sm_extra_grab,
                               transitions={'succeeded': 'UP',
                                            'empty': 'RETRY_GRAB_COUNTER'})
        smach.StateMachine.add('UP',
                               common_states.WaypointState(shared, lambda cur: cur.depth(.3)))
        smach.StateMachine.add('RETRY_GRAB_COUNTER', common_states.CounterState(1),
                               transitions={'succeeded': 'DESCEND_GRAB',
                                            'exceeded': 'APPROACH'})
    return sm

def make_grabber_drop(shared):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(2)))
        smach.Sequence.add('OPEN_GRABBER',
                           subjugator_states.OpenGrabberState())
        smach.Sequence.add('UP_DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(.5)))                           
    return sm
        
missions.register_factory('grabber', make_grabber)
missions.register_factory('grabber_drop', make_grabber_drop)
