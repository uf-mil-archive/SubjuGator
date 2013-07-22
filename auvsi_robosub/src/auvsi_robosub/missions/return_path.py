from uf_smach import common_states, object_finder_states, legacy_vision_states, missions

import numpy
import smach

def make_return_path(shared):
    search_sm = smach.Concurrence(['succeeded', 'failed', 'preempted'],
                                  default_outcome='failed',
                                  outcome_map={'succeeded': {'WAIT': 'succeeded'}},
                                  child_termination_cb=lambda so: True)
    with search_sm:
        smach.Concurrence.add('PATTERN',
                              common_states.WaypointSeriesState(shared,
                                                                [lambda cur: cur.left(2),
                                                                 lambda cur: cur.forward(2),
                                                                 lambda cur: cur.right(4),
                                                                 lambda cur: cur.backward(2)],
                                                                speed=.3))
        smach.Concurrence.add('WAIT',
                              legacy_vision_states.WaitForObjectsState(shared,
                                                                       'find2_down_camera', 'pipe'))                                  
    sm = smach.StateMachine(['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('RETURN_WAYPOINT',
                               common_states.ReturnToWaypointState(shared, 'last_path'),
                               transitions={'succeeded': 'SEARCH'})
        smach.StateMachine.add('SEARCH', search_sm)        
    return sm

missions.register_factory('return_path', make_return_path)

