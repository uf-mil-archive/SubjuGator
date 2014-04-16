from __future__ import division

import json
import math
import traceback

import numpy
from twisted.internet import defer

from txros import action, util, tf

from uf_common.msg import MoveToAction, MoveToGoal, PoseTwistStamped, Float64Stamped
from legacy_vision.msg import FindAction, FindGoal
from uf_common import orientation_helpers
from tf import transformations


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
        self._tf_listener = tf.TransformListener(self._node_handle)
        self._dvl_range_sub = self._node_handle.subscribe('dvl/range', Float64Stamped)
        
        yield self._trajectory_sub.get_next_message()
        
        defer.returnValue(self)
    
    @property
    def _pose(self):
        return orientation_helpers.PoseEditor.from_PoseTwistStamped(
            self._trajectory_sub.get_last_message())
    
    @property
    def move(self):
        return _PoseProxy(self, self._pose)
    
    def get_dvl_range(self):
        return self._dvl_range_sub.get_next_message()
    
    @util.cancellableInlineCallbacks
    def visual_align(self, camera, object_name, distance_estimate):
        goal_mgr = self._camera_action_clients[camera].send_goal(FindGoal(
            object_names=[object_name],
        ))
        start_pose = self._pose
        start_map_transform = tf.Transform(
            start_pose.position, start_pose.orientation)
        try:
            while True:
                feedback = yield goal_mgr.get_feedback()
                res = map(json.loads, feedback.targetreses[0].object_results)
                
                try:
                    transform = yield self._tf_listener.get_transform('/base_link',
                        feedback.header.frame_id, feedback.header.stamp)
                    map_transform = yield self._tf_listener.get_transform('/map',
                        '/base_link', feedback.header.stamp)
                except Exception:
                    traceback.print_exc()
                    continue
                
                if not res: continue
                obj = res[0]
                
                ray_start_camera = numpy.array([0, 0, 0])
                ray_dir_camera = numpy.array(map(float, obj['center']))
                obj_dir_camera = numpy.array(map(float, obj['direction']))
                
                ray_start_world = map_transform.transform_point(
                    transform.transform_point(ray_start_camera))
                ray_dir_world = map_transform.transform_vector(
                    transform.transform_vector(ray_dir_camera))
                obj_dir_world = map_transform.transform_vector(
                    transform.transform_vector(obj_dir_camera))
                
                axis_camera = [0, 0, 1]
                axis_body = transform.transform_vector(axis_camera)
                axis_world = start_map_transform.transform_vector(axis_body)
                
                # project ray onto plane defined by distance_estimate
                # from start_pose.position along axis_world
                
                plane_point = distance_estimate * axis_world + start_pose.position
                plane_vector = axis_world
                
                x = plane_vector.dot(ray_start_world - plane_point) / plane_vector.dot(ray_dir_world)
                object_pos = ray_start_world - ray_dir_world * x
                
                desired_pos = object_pos - start_map_transform.transform_vector(transform.transform_point(ray_start_camera))
                desired_pos = desired_pos - axis_world * axis_world.dot(desired_pos - start_pose.position)
                
                error_pos = desired_pos - map_transform.transform_point([0, 0, 0])
                error_pos = error_pos - axis_world * axis_world.dot(error_pos)
                
                print desired_pos, numpy.linalg.norm(error_pos)/3e-2
                
                if numpy.linalg.norm(error_pos) < 3e-2: # 3 cm
                    # yield go to pos and angle
                    
                    direction_symmetry = int(obj['direction_symmetry'])
                    dangle = 2*math.pi/direction_symmetry
                    
                    def rotate(x, angle):
                        return transformations.rotation_matrix(angle, axis_world)[:3, :3].dot(x)
                    for sign in [-1, +1]:
                        while rotate(obj_dir_world, sign*dangle).dot(start_pose.forward_vector) > obj_dir_world.dot(start_pose.forward_vector):
                            obj_dir_world = rotate(obj_dir_world, sign*dangle)
                    
                    yield (self.move
                        .set_position(desired_pos)
                        .look_at_rel_without_pitching(obj_dir_world)
                        .go())
                    
                    return
                
                # go towards desired position
                self._moveto_action_client.send_goal(
                    start_pose.set_position(desired_pos).as_MoveToGoal()).forget()
        finally:
            goal_mgr.cancel()

_subs = {}
def get_sub(node_handle):
    if node_handle not in _subs:
        _subs[node_handle] = _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    return _subs[node_handle]
