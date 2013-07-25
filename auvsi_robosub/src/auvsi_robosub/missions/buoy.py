import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import constants
from uf_smach import common_states, object_finder_states, legacy_vision_states, missions
from object_finder.msg import TargetDesc

import numpy
import smach

SEARCH_WIDTH = 4 # How far to strafe
SEARCH_ADVANCE = 1.5 # How far to move forward after strafing

buoy_desc = TargetDesc()
buoy_desc.type = TargetDesc.TYPE_SPHERE
buoy_desc.sphere_radius = 4*.0254 # 4 in
buoy_desc.prior_distribution.pose.orientation.w = 1
buoy_desc.min_dist = 2
buoy_desc.max_dist = 6

def make_buoy(shared):
    buoy_sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with buoy_sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(2.5)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_BUOYS',
                           object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                    lambda: [buoy_desc], .90),
                           transitions={'timeout': 'failed'})
        smach.Sequence.add('APPROACH_BUOY',
                           object_finder_states.ApproachObjectState(shared,
                                                                    'find_forward',
                                                                    'forward_camera', 2))
        smach.Sequence.add('BUMP',
                           common_states.WaypointSeriesState(shared,
                                                             [lambda cur: cur.forward(2.5),
                                                              lambda cur: cur.backward(.5)]))
        smach.Sequence.add('OVER',
                           common_states.WaypointSeriesState(shared,
                                                             [lambda cur: cur.depth(.5),
                                                              lambda cur: cur.forward(2.5)]))

    search_pattern_sm = smach.StateMachine(['preempted'])
    with search_pattern_sm:
        smach.StateMachine.add('START',
                               common_states.WaypointSeriesState(shared,
                                                                 [lambda cur: cur.depth(constants.PIPE_DEPTH),
                                                                  lambda cur: cur.left(SEARCH_WIDTH/2)],
                                                                 speed=constants.PIPE_SPEED),
                               transitions={'succeeded': 'SEARCH'})
        smach.StateMachine.add('SEARCH',
                               common_states.WaypointSeriesState(shared,
                                                                 [lambda cur: cur.right(SEARCH_WIDTH),
                                                                  lambda cur: cur.forward(SEARCH_ADVANCE),
                                                                  lambda cur: cur.left(SEARCH_WIDTH),
                                                                  lambda cur: cur.forward(SEARCH_ADVANCE)],
                                                                 speed=constants.PIPE_SPEED),
                               transitions={'succeeded': 'SEARCH'})

    search_sm = smach.Concurrence(['succeeded', 'failed', 'preempted'],
                                  default_outcome='failed',
                                  outcome_map={'succeeded': {'WAIT': 'succeeded'}},
                                  child_termination_cb=lambda so: True)
    with search_sm:
        smach.Concurrence.add('PATTERN', search_pattern_sm)
        smach.Concurrence.add('WAIT',
                              legacy_vision_states.WaitForObjectsState(shared,
                                                                       'find2_down_camera', 'pipe'))
        
    sm = smach.StateMachine(['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('BUOY', buoy_sm,
                               transitions={'succeeded': 'SEARCH',
                                            'failed': 'SEARCH'})
        smach.StateMachine.add('SEARCH', search_sm)
    return sm

missions.register_factory('buoy', make_buoy)
