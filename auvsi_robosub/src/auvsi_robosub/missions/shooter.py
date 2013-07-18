import roslib; roslib.load_manifest('uf_smach')
from auvsi_robosub import subjugator_states
from uf_smach import missions, common_states, object_finder_states
from uf_common.orientation_helpers import PoseEditor
from object_finder.msg import TargetDesc
from uf_common.msg import PoseTwistStamped
from geometry_msgs.msg import Quaternion

import numpy
import smach

DEPTH = 1.5

#BOARD_DIST = 2.5 # box-centering distance
BOARD_DIST = 2
HEXAGON_DIST = .5 # hexagon-searching distance
SHOOT_DIST = 0.15 # shooting distance

SIZE = 'small'
COLORS = ['red', 'blue']

def shooter_desc_cb():
    traj = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
    desc = TargetDesc()
    desc.type = TargetDesc.TYPE_OBJECT
    desc.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/shooter.obj')
    desc.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    desc.disallow_yawing = True
    desc.min_dist = BOARD_DIST-1
    desc.max_dist = BOARD_DIST+3
    return [desc]

def hexagon_desc_cb(size, color):
    traj = PoseEditor.from_PoseTwistStamped_topic('/trajectory')
    desc = TargetDesc()
    desc.type = TargetDesc.TYPE_OBJECT
    desc.object_filename = roslib.packages.resource_file('auvsi_robosub', 'models', '2013/shooter_%sin_%s_hexagon.obj' % (7 if size == 'small' else 12, color))
    desc.prior_distribution.pose.orientation = Quaternion(*traj.turn_left_deg(180).orientation)
    cov = numpy.zeros((6, 6))
    a = numpy.array([traj.forward_vector]).T * 100
    cov[3:, 3:] += a.dot(a.T)
    desc.prior_distribution.covariance = cov.flatten()
    desc.min_dist = HEXAGON_DIST*.9
    desc.max_dist = HEXAGON_DIST/.9
    desc.allow_rolling = True
    desc.disallow_yawing = True
    return [desc]
    
def make_shooter(shared):
    # Create a SMACH state machine
    sm_approach = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_approach:
        smach.Sequence.add('DEPTH',
                           common_states.WaypointState(shared, lambda cur: cur.depth(DEPTH)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.4, 0, 0])))
        smach.Sequence.add('WAIT_SHOOTER',
                           object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                    shooter_desc_cb, .95),
                           transitions={'timeout': 'failed'})
    sm_shoots = []
    for color, shooter in zip(COLORS, ['left', 'right']):
        sm_shoot = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
        sm_shoots.append(sm_shoot)
        with sm_shoot:
            smach.Sequence.add('APPROACH_SHOOTER',
                               object_finder_states.ApproachObjectState(shared,
                                                                        'find_forward', 'forward_camera',
                                                                        BOARD_DIST, marker=color))
            smach.Sequence.add('OPEN_LOOP_FORWARD',
                               common_states.WaypointState(shared,
                                                           lambda cur: cur.forward(BOARD_DIST-HEXAGON_DIST)))
            smach.Sequence.add('WAIT_HEXAGON',
                               object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                        lambda: hexagon_desc_cb(SIZE, color),
                                                                        .99),
                               transitions={'timeout': 'failed'})
            smach.Sequence.add('APPROACH_HEXAGON',
                               object_finder_states.ApproachObjectState(shared,
                                                                        'find_forward', 'forward_camera',
                                                                        HEXAGON_DIST))
            smach.Sequence.add('OPEN_LOOP_FORWARD2',
                               common_states.WaypointState(shared,
                                                           lambda cur: cur.forward(HEXAGON_DIST-SHOOT_DIST)\
                                                                          .relative([0, .12, .18]
                                                                                    if shooter == 'left' else
                                                                                    [0, -.12, .18])))
            smach.Sequence.add('SHOOT', subjugator_states.ShootTorpedoState(shooter))

    sm_retreat = smach.Sequence(['succeeded', 'failed', 'preempted'], 'succeeded')
    with sm_retreat:
        smach.Sequence.add('RETREAT',
                           common_states.WaypointState(shared,
                                                       lambda cur: cur.backward(BOARD_DIST+1).depth(DEPTH)))
        smach.Sequence.add('APPROACH',
                           common_states.VelocityState(shared, numpy.array([.2, 0, 0])))
        smach.Sequence.add('WAIT_SHOOTER',
                           object_finder_states.WaitForObjectsState(shared, 'find_forward',
                                                                    shooter_desc_cb, .9),
                           transitions={'timeout': 'failed'})
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
                           
missions.register_factory('shooter', make_shooter)
