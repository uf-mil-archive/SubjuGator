from __future__ import division

import json
import math

import numpy
from twisted.internet import defer

from txros import action, util

from uf_common.msg import MoveToAction, MoveToGoal, PoseTwistStamped
from legacy_vision.msg import FindAction, FindGoal
from uf_common import orientation_helpers


class _PoseProxy(object):
    def __init__(self, sub, pose):
        self._sub = sub
        self._pose = pose
    
    def __getattr__(self, name):
        def _(*args, **kwargs):
            return _PoseProxy(self._sub, getattr(self._pose, name)(*args, **kwargs))
        return _
    
    def go(self, *args, **kwargs):
        return self._sub._moveto_action_client.send_goal(
            self._pose.as_MoveToGoal(*args, **kwargs)).get_result()

class _Sub(object):
    def __init__(self, node_handle):
        self._node_handle = node_handle
    
    @util.cancellableInlineCallbacks
    def _init(self):
        self._trajectory_sub = self._node_handle.subscribe('trajectory', PoseTwistStamped)
        self._moveto_action_client = action.ActionClient(self._node_handle, 'moveto', MoveToAction)
        self._camera_action_clients = dict(
            forward=action.ActionClient(self._node_handle, 'find2_forward_camera', FindAction),
            down=action.ActionClient(self._node_handle, 'find2_down_camera', FindAction),
        )
        
        yield self._trajectory_sub.get_next_message()
        
        defer.returnValue(self)
    
    @property
    def _pose(self):
        return orientation_helpers.PoseEditor.from_PoseTwistStamped(
            self._trajectory_sub.get_last_message())
    
    @property
    def move(self):
        return _PoseProxy(self, self._pose)
    
    @util.cancellableInlineCallbacks
    def visual_align(self, camera, object_name):
        goal_mgr = self._camera_action_clients[camera].send_goal(FindGoal(
            object_names=[object_name],
        ))
        try:
            while True:
                feedback = yield goal_mgr.get_feedback()
                res = map(json.loads, feedback.targetreses[0].object_results)
                
                if not res: continue
                obj = res[0]
                
                center = numpy.array(map(float, obj['center']))
                center = center / numpy.linalg.norm(center)
                
                angle = math.atan2(
                    -float(obj['direction'][0]),
                    -float(obj['direction'][1]))
                
                forward_vel = -float(obj['center'][1]) * 0.5
                left_vel = -float(obj['center'][0]) * 0.5
                
                print angle, forward_vel, left_vel
                
                if center.dot([0, 0, 1]) > math.cos(math.radians(1)):
                    break
                
                self._moveto_action_client.send_goal(
                    orientation_helpers.PoseEditor.from_PoseTwistStamped(
                        self._pose).as_MoveToGoal(linear=[forward_vel, left_vel, 0])).forget()
            
            direction_symmetry = int(obj['direction_symmetry'])
            dangle = 2*math.pi/direction_symmetry
            
            while abs(angle + dangle) < abs(angle):
                angle += dangle
            while abs(angle - dangle) < abs(angle):
                angle -= dangle
            
            print 'a'
            yield self.move.yaw_left(angle).go()
            print 'b'
        finally:
            print 1
            self._moveto_action_client.send_goal(
                orientation_helpers.PoseEditor.from_PoseTwistStamped(
                    self._pose).as_MoveToGoal()).forget()
            #goal_mgr.cancel()
            print 2

_subs = {}
def get_sub(node_handle):
    if node_handle not in _subs:
        _subs[node_handle] = _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    return _subs[node_handle]
