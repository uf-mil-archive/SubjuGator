from uf_smach import common_states, missions

import rospy
import smach
import smach_ros

def make_test(shared, config):
    sm = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(2)))
        for i in xrange(config['iterations']):
            smach.Sequence.add('MOVE %d' % i,
                               common_states.WaypointSeriesState(shared, 
                                                                 [lambda cur: cur.forward(2),
                                                                  lambda cur: cur.turn_left_deg(90)]))
    return sm

missions.register_factory('test', make_test)
