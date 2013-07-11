from uf_smach import common_states, missions

import rospy
import smach
import smach_ros

def make_test(shared, iterations):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(2)))
        for i in xrange(iterations):
            smach.Sequence.add('UNRELIABLE %d' %i,
                               common_states.UnreliableState(.75))
            smach.Sequence.add('MOVE %d' % i,
                               common_states.WaypointSeriesState(shared, 
                                                                 [lambda cur: cur.forward(2),
                                                                  lambda cur: cur.turn_left_deg(90)]))
    return sm

missions.register_factory('test', lambda shared: make_test(shared, 6))
missions.register_factory('short test', lambda shared: make_test(shared, 2))
