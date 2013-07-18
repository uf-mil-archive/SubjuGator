from auvsi_robosub import subjugator_states
from uf_smach import common_states, legacy_vision_states, missions

import numpy
import smach

DROP_ORDER = ['37', '98']

def unit_vector(x):
    return x/numpy.linalg.norm(x)

def select_image_text_or_most_central(image_text):
    def _(results, traj_start, (tf_p, tf_q)):
        if all('image_text' in result for result in results):
            matches = [result for result in results if result['image_text'] == image_text]
            assert len(matches) == 1 # guaranteed by BinsFinder
            return matches[0]
        else:
            return max(results, key=lambda result:
                unit_vector(numpy.array(map(float, result['center']))).dot([0, 0, 1])
            )
    return _

def make_bins(shared):
    sm_approach = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_approach:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(.4)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared,
                                                       numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_ALL',
                           legacy_vision_states.WaitForObjectsState(shared,
                                                                    'find2_down_camera', 'bins/all'),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('EXTRA_FORWARD',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.forward(.5)))
        
    sm_center = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_center:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(.4)))
        smach.Sequence.add('WAIT_ALL',
                           legacy_vision_states.WaitForObjectsState(shared,
                                                                    'find2_down_camera', 'bins/all',
                                                                    timeout=10),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('CENTER_ALL',
                           legacy_vision_states.CenterObjectState(shared, 'find2_down_camera'))
        smach.Sequence.add('ALIGN_ALL',
                           legacy_vision_states.AlignObjectState(shared, 'find2_down_camera',
                                                                 body_vec_align=[0, 1, 0]))
        smach.Sequence.add('CENTER_ALL_2',
                           legacy_vision_states.CenterObjectState(shared, 'find2_down_camera'))


    sm_drops = []
    for bin_string in DROP_ORDER:
        sm_drop = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
        sm_drops.append(sm_drop)
        
        with sm_drop:
        # this could be faster if CenterObjectState let you descend at the same time
            smach.Sequence.add('WAIT_SINGLE',
                               legacy_vision_states.WaitForObjectsState(shared,
                                                                        'find2_down_camera',
                                                                        'bins/single',
                                                                        timeout=10),
                               transitions={'timeout': 'failed'})
            selector = select_image_text_or_most_central(bin_string)
            smach.Sequence.add('APPROACH_SINGLE',
                               legacy_vision_states.CenterApproachObjectState(shared,
                                                                              'find2_down_camera',
                                                                              desired_scale=9000,
                                                                              selector=selector))
            smach.Sequence.add('DOWN',
                               common_states.WaypointState(shared, lambda cur: cur.down(.5).backward(.1)))
            smach.Sequence.add('DROP',
                               subjugator_states.DropBallState())

    sm_pipe = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_pipe:
        smach.Sequence.add('TURN_DEPTH',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.depth(1).turn_right_deg(90))) # Should be left at transdec
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
                               transitions={'succeeded': 'CENTER_1'})
        smach.StateMachine.add('CENTER_1', sm_center,
                               transitions={'succeeded': 'DROP_1',
                                            'failed': 'APPROACH'})
        smach.StateMachine.add('DROP_1', sm_drops[0],
                               transitions={'succeeded': 'CENTER_2',
                                            'failed': 'RETRY_CENTER_1'})
        smach.StateMachine.add('RETRY_CENTER_1',
                               common_states.CounterState(1),
                               transitions={'succeeded': 'CENTER_1',
                                            'exceeded': 'CENTER_2'})
        smach.StateMachine.add('CENTER_2', sm_center,
                               transitions={'succeeded': 'DROP_2'})
        smach.StateMachine.add('DROP_2', sm_drops[1],
                               transitions={'succeeded': 'FIND_PIPE',
                                            'failed': 'RETRY_CENTER_2'})
        smach.StateMachine.add('RETRY_CENTER_2',
                               common_states.CounterState(1),
                               transitions={'succeeded': 'CENTER_2',
                                            'exceeded': 'FIND_PIPE'})
        smach.StateMachine.add('FIND_PIPE', sm_pipe)        
    return sm

missions.register_factory('bins', make_bins)
