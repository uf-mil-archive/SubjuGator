from __future__ import division

import json
import math
import traceback
import time

import numpy
from twisted.internet import defer

from txros import action, util, tf

import genpy
from std_msgs.msg import Header, Bool
from uf_common.msg import MoveToAction, PoseTwistStamped, Float64Stamped
from legacy_vision import msg as legacy_vision_msg
from object_finder import msg as object_finder_msg
from actuator_driver.srv import PulseValve, PulseValveRequest, SetValve, SetValveRequest
from uf_common import orientation_helpers
from tf import transformations
from c3_trajectory_generator.srv import SetDisabled, SetDisabledRequest
from odom_estimator.srv import SetIgnoreMagnetometer, SetIgnoreMagnetometerRequest
from hydrophones.msg import ProcessedPing
from geometry_msgs.msg import WrenchStamped, Point, PointStamped
from sub8_vision_arbiter.msg import *

X_RES = 640
Y_RES = 480
CAMERA_X_CENTER = X_RES/2
CAMERA_Y_CENTER = Y_RES/2

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
    
    def go_trajectory(self, *args, **kwargs):
        self._sub._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs))

class _Sub(object):
    def __init__(self, node_handle):
        self._node_handle = node_handle
    
    @util.cancellableInlineCallbacks
    def _init(self, need_trajectory=True):
        self._trajectory_sub = self._node_handle.subscribe('trajectory', PoseTwistStamped)
        self._trajectory_pub = self._node_handle.advertise('trajectory', PoseTwistStamped)
        self._trajectory_generator_set_disabled_service = self._node_handle.get_service_client('c3_trajectory_generator/set_disabled', SetDisabled)
        self._moveto_action_client = action.ActionClient(self._node_handle, 'moveto', MoveToAction)
        self._camera_2d_action_clients = dict(
            forward=action.ActionClient(self._node_handle, 'find2_forward_camera', legacy_vision_msg.FindAction),
            down=action.ActionClient(self._node_handle, 'find2_down_camera', legacy_vision_msg.FindAction),
        )
        self._camera_3d_action_clients = dict(
            forward=action.ActionClient(self._node_handle, 'find_forward', object_finder_msg.FindAction),
            down=action.ActionClient(self._node_handle, 'find_down', object_finder_msg.FindAction),
        )
        self._tf_listener = tf.TransformListener(self._node_handle)
        self._dvl_range_sub = self._node_handle.subscribe('dvl/range', Float64Stamped)
        self._pulse_valve_service = self._node_handle.get_service_client(
            'actuator_driver/pulse_valve', PulseValve)
        self._set_valve_service = self._node_handle.get_service_client(
            'actuator_driver/set_valve', SetValve)
        self._set_ignore_magnetometer_service = self._node_handle.get_service_client(
            'odom_estimator/set_ignore_magnetometer', SetIgnoreMagnetometer)
        self._hydrophones_processed_sub = self._node_handle.subscribe('hydrophones/processed', ProcessedPing)
        self._wrench_sub = self._node_handle.subscribe('wrench', WrenchStamped)

        self._green_buoy_sub = self._node_handle.subscribe("green_buoy_vision" , Point)      
        self._red_marker_sub = self._node_handle.subscribe("red_marker_vision" , Point)
        self._train_sub = self._node_handle.subscribe("train" , Point)
        self._tracks_sub = self._node_handle.subscribe("tracks" , Point)
        self._torpedo_center = self._node_handle.subscribe("torpedo/center", Point)
        self._torpedo_TL = self._node_handle.subscribe("torpedo/TL", Point)
        self._handle_location_sub = self._node_handle.subscribe("handle", Point)
        self._vision_simulator = self._node_handle.subscribe("align_test_point", Point)
        self._delorean_sub = self._node_handle.subscribe("delorean", Point)

        self._vision_control_sub = self._node_handle.advertise('vision_arbiter', Point)
        self._vision_control_sub = self._node_handle.advertise('vision_arbiter', Point)
        self._begin = self._node_handle.subscribe('begin', Bool)

        if(need_trajectory == True):
            yield self._trajectory_sub.get_next_message()

        
        defer.returnValue(self)
    
    @property
    def pose(self):
        return orientation_helpers.PoseEditor.from_PoseTwistStamped(
            self._trajectory_sub.get_last_message())
    
    @property
    def move(self):
        return _PoseProxy(self, self.pose)
    
    @util.cancellableInlineCallbacks
    def get_dvl_range(self):
        msg = yield self._dvl_range_sub.get_next_message()
        defer.returnValue(msg.data)

    @util.cancellableInlineCallbacks
    def get_begin(self):
        msg = yield self._begin.get_next_message()
        defer.returnValue(msg.data)
    
    @util.cancellableInlineCallbacks
    def visual_align(self, camera, object_name, distance_estimate, selector=lambda items, body_tf: items[0], turn=True, angle=0, orient_away_from=None, one_shot=False):
        goal_mgr = self._camera_2d_action_clients[camera].send_goal(legacy_vision_msg.FindGoal(
            object_names=[object_name],
        ))
        start_pose = self.pose
        start_map_transform = tf.Transform(
            start_pose.position, start_pose.orientation)
        move_goal_mgr = None
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
                obj = selector(res, transform)
                if obj is None: continue
                
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
                
                if numpy.linalg.norm(error_pos) < 3e-2 or one_shot: # 3 cm
                    if turn:
                        direction_symmetry = int(obj['direction_symmetry'])
                        dangle = 2*math.pi/direction_symmetry
                        
                        def rotate(x, angle):
                            return transformations.rotation_matrix(angle, axis_world)[:3, :3].dot(x)
                        obj_dir_world = rotate(obj_dir_world, angle)
                        for sign in [-1, +1]:
                            f = lambda obj_dir_world: obj_dir_world.dot(start_pose.forward_vector)
                            if orient_away_from is not None:
                                f = lambda obj_dir_world: obj_dir_world.dot(object_pos - orient_away_from)
                            while f(rotate(obj_dir_world, sign*dangle)) > f(obj_dir_world):
                                obj_dir_world = rotate(obj_dir_world, sign*dangle)
                        
                        yield (self.move
                            .set_position(desired_pos)
                            .look_at_rel_without_pitching(obj_dir_world)
                            .go())
                    else:
                        yield (self.move
                            .set_position(desired_pos)
                            .go())
                    
                    return
                
                # go towards desired position
                move_goal_mgr = self._moveto_action_client.send_goal(
                    start_pose.set_position(desired_pos).as_MoveToGoal(speed=0.1))
        finally:
            yield goal_mgr.cancel()
            if move_goal_mgr is not None: yield move_goal_mgr.cancel()
    
    @util.cancellableInlineCallbacks
    def visual_approach(self, camera, object_name, size_estimate, desired_distance, selector=lambda items, body_tf: items[0]):
        goal_mgr = self._camera_2d_action_clients[camera].send_goal(legacy_vision_msg.FindGoal(
            object_names=[object_name],
        ))
        start_pose = self.pose
        start_map_transform = tf.Transform(
            start_pose.position, start_pose.orientation)
        move_goal_mgr = None
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
                obj = selector(res, transform)
                if obj is None: continue
                
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
                
                distance_estimate = size_estimate / (2 * numpy.linalg.norm(obj_dir_camera))
                print distance_estimate
                
                object_pos = ray_start_world + ray_dir_world * distance_estimate
                print object_pos
                
                desired_pos = object_pos - axis_world * desired_distance
                
                error_pos = desired_pos - map_transform.transform_point([0, 0, 0])
                
                print desired_pos, numpy.linalg.norm(error_pos)/6e-2
                
                if numpy.linalg.norm(error_pos) < 6e-2: # 6 cm
                    yield (self.move
                        .set_position(desired_pos)
                        .go())
                    
                    break
                    # defer.returnValue(obj)
                
                # go towards desired position
                move_goal_mgr = self._moveto_action_client.send_goal(
                    start_pose.set_position(desired_pos).as_MoveToGoal(speed=0.1))
        finally:
            yield goal_mgr.cancel()
            if move_goal_mgr is not None: yield move_goal_mgr.cancel()

    @util.cancellableInlineCallbacks
    def change_current_vision(self, recovery_vision = False,
                                    torpedo_vision = False,
                                    torpedo_area_vision = False,
                                    train_vision = False,
                                    path_vision = False,
                                    buoys_vision = False,
                            ):
        msg = vision_arbiter()
        msg.recovery_vision = recovery_vision
        msg.torpedo_vision = torpedo_vision
        msg.torpedo_area_vision = torpedo_area_vision
        msg.tracks_vision = train_vision
        msg.path_vision = path_vision
        msg.buoys_vision = buoys_vision
        self._vision_control_sub.publish(msg)

    @util.cancellableInlineCallbacks
    def get_target_location(self, target):
        if target == 'delorean':
            msg = yield self._delorean_sub.get_next_message()
            defer.returnValue(msg)
        if target == 'train':
            msg = yield self._train_sub.get_next_message()
            defer.returnValue(msg)
        if target == 'tracks':
            msg = yield self._tracks_sub.get_next_message()
            defer.returnValue(msg)
        if target == 'handle':
            msg = yield self._handle_location_sub.get_next_message()
            defer.returnValue(msg)
        if target == 'center':
            msg = yield self._torpedo_center.get_next_message()
            defer.returnValue(msg)
        if target == 'top_left':
            msg = yield self._torpedo_TL.get_next_message()
            defer.returnValue(msg)
        if target == "vis_simulator":
            msg = yield self._vision_simulator.get_next_message()
            defer.returnValue(msg)

        print 'Invalid target ', target
        assert False
    
    @util.cancellableInlineCallbacks
    def orient_and_align(self, linear_align_target, orientation_align_target, move_scale, angular_tolerance, orientation_tolerance):

        def calc_y_angle(opp_input):
            adjacent = CAMERA_Y_CENTER
            opposite = opp_input
            hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
            arcsin = math.acos(opposite/hypotenuse)
            return abs(.80 - arcsin)

        def calc_x_angle(opp_input): 
            adjacent = CAMERA_X_CENTER
            opposite = opp_input
            hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
            arcsin = math.acos(opposite/hypotenuse)
            return abs(.80 - arcsin)

        x_location = 1
        y_location = 1
        orientation = 1

        while x_location > angular_tolerance and y_location > angular_tolerance:

            center_location = yield self.get_target_location(linear_align_target)
            x_location = center_location.x
            y_location = center_location.y
            orientation = center_location.z

            x_move = calc_x_angle(x_location) * move_scale
            y_move = calc_y_angle(y_location) * move_scale

            if x_location < CAMERA_X_CENTER: 
                print "Moving left", x_move
                yield self.move.left(x_move)
            if x_location > CAMERA_X_CENTER: 
                print "Moving right", x_move
                yield self.move.right(x_move)
            if y_location < CAMERA_Y_CENTER: 
                print "Moving up", y_move
                yield self.move.up(y_move)
            if y_location > CAMERA_Y_CENTER:
                print "Moving down", y_move
                yield self.move.down(y_move)

        target_orientation = yield self.get_target_location(orientation_align_target)

        goal_move = abs(90 - target_orientation.z)
        if goal_move > 180: goal_move = 360 - goal_move

        if target_orientation.z > 90 and target_orientation.z < 270:
            print "Move Left", goal_move
            yield self.move.yaw_left(goal_move).go()
        else: 
            print "Move right", goal_move
            yield self.move.yaw_right(goal_move).go()  

    @util.cancellableInlineCallbacks
    def align(self, align_target, move_scale, angular_tolerance):

        def calc_y_angle(opp_input):
            adjacent = CAMERA_Y_CENTER
            opposite = opp_input
            hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
            arcsin = math.acos(opposite/hypotenuse)
            return abs(.80 - arcsin)

        def calc_x_angle(opp_input):
            adjacent = CAMERA_X_CENTER
            opposite = opp_input
            hypotenuse = math.sqrt(adjacent*adjacent + opposite*opposite)
            arcsin = math.acos(opposite/hypotenuse)
            return abs(.80 - arcsin)

        x_location = 1
        y_location = 1

        while x_location > angular_tolerance and y_location > angular_tolerance:

            center_location = yield self.get_target_location(align_target)
            x_location = center_location.x
            y_location = center_location.y

            x_move = calc_x_angle(x_location) * move_scale
            y_move = calc_y_angle(y_location) * move_scale

            if x_location < CAMERA_X_CENTER: 
                print "Moving left", x_move
                yield self.move.left(x_move)
            if x_location > CAMERA_X_CENTER: 
                print "Moving right", x_move
                yield self.move.right(x_move)
            if y_location < CAMERA_Y_CENTER: 
                print "Moving up", y_move
                yield self.move.up(y_move)
            if y_location > CAMERA_Y_CENTER:
                print "Moving down", y_move
                yield self.move.down(y_move)


    @util.cancellableInlineCallbacks
    def get_green_buoy(self):
        msg = yield self._green_buoy_sub.get_next_message()
        defer.returnValue(msg)

    @util.cancellableInlineCallbacks
    def get_red_marker(self):
        msg = yield self._red_marker_sub.get_next_message()
        defer.returnValue(msg)

    @util.cancellableInlineCallbacks
    def visual_approach_3d(self, camera, distance, targetdesc, loiter_time=0):
        goal_mgr = self._camera_3d_action_clients[camera].send_goal(object_finder_msg.FindGoal(
            header=Header(
                frame_id='/map',
            ),
            targetdescs=[targetdesc],
        ))
        start_pose = self.pose
        
        move_goal_mgr = None
        try:
            last_good_pos = None
            loiter_start = None
            
            while True:
                feedback = yield goal_mgr.get_feedback()
                targ = feedback.targetreses[0]
                
                if targ.P > 0.25:
                    last_good_pos = orientation_helpers.xyz_array(targ.pose.position)
                
                print targ.P
                
                if last_good_pos is not None:
                    desired_pos = start_pose.set_position(last_good_pos).backward(distance).position
                    
                    print ' '*20, numpy.linalg.norm(desired_pos - self.pose.position)
                    
                    if numpy.linalg.norm(desired_pos - self.pose.position) < 0.5:
                        if loiter_start is None:
                            loiter_start = time.time()
                        
                        if time.time() > loiter_start + loiter_time:
                            yield self._moveto_action_client.send_goal(
                                start_pose.set_position(desired_pos).as_MoveToGoal()).get_result()
                            return
                    
                    move_goal_mgr = self._moveto_action_client.send_goal(
                        start_pose.set_position(desired_pos).as_MoveToGoal(speed=0.5))
        finally:
            yield goal_mgr.cancel()
            if move_goal_mgr is not None: yield move_goal_mgr.cancel()
    
    @util.cancellableInlineCallbacks
    def set_trajectory_generator_enable(self, enabled):
        yield self._trajectory_generator_set_disabled_service(SetDisabledRequest(
            disabled=not enabled, 

        ))


    @util.cancellableInlineCallbacks
    def raise_down_grabber(self):
        yield self._set_valve_service(SetValveRequest(valve=3, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=0, opened=True))
    @util.cancellableInlineCallbacks
    def lower_down_grabber(self):
        yield self._set_valve_service(SetValveRequest(valve=0, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=3, opened=True))
    @util.cancellableInlineCallbacks
    def open_down_grabber(self):
        yield self._set_valve_service(SetValveRequest(valve=1, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=2, opened=True))
    @util.cancellableInlineCallbacks
    def close_down_grabber(self):
        yield self._set_valve_service(SetValveRequest(valve=2, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=1, opened=True))
    
    @util.cancellableInlineCallbacks
    def open_gripper(self):
        yield self._set_valve_service(SetValveRequest(valve=3, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=1, opened=True))
    @util.cancellableInlineCallbacks
    def close_gripper(self):
        yield self._set_valve_service(SetValveRequest(valve=1, opened=False))
        yield util.sleep(.3)
        yield self._set_valve_service(SetValveRequest(valve=3, opened=True))
    
    @util.cancellableInlineCallbacks
    def drop_ball(self):
        yield self._pulse_valve_service(PulseValveRequest(valve=1, duration=genpy.Duration(1.5)))
    
    @util.cancellableInlineCallbacks
    def fire_left_torpedo(self):
        yield self._pulse_valve_service(PulseValveRequest(valve=4, duration=genpy.Duration(.3)))
    @util.cancellableInlineCallbacks
    def fire_right_torpedo(self):
        yield self._pulse_valve_service(PulseValveRequest(valve=5, duration=genpy.Duration(.3)))
    
    @util.cancellableInlineCallbacks
    def set_ignore_magnetometer(self, ignore):
        yield self._set_ignore_magnetometer_service(SetIgnoreMagnetometerRequest(ignore=ignore))
    
    @util.cancellableInlineCallbacks
    def get_processed_ping(self, frequency):
        while True:
            msg = yield self._hydrophones_processed_sub.get_next_message()
            if abs(msg.freq - frequency) < 1.5e3:
                defer.returnValue(msg)
    
    @util.cancellableInlineCallbacks
    def hydrophone_align(self, frequency):
        start_pose = self.pose
        start_map_transform = tf.Transform(
            start_pose.position, start_pose.orientation)
        orientation = start_pose.orientation
        move_goal_mgr = None
        good = []
        try:
            while True:
                feedback = yield self.get_processed_ping(frequency)
                print feedback
                bottom_z = self.pose.position[2] - (yield self.get_dvl_range())
                print 'bottom_z:', bottom_z
                
                try:
                    transform = yield self._tf_listener.get_transform('/base_link',
                        feedback.header.frame_id, feedback.header.stamp)
                    map_transform = yield self._tf_listener.get_transform('/map',
                        '/base_link', feedback.header.stamp)
                except Exception:
                    traceback.print_exc()
                    continue
                
                ray_start_sensor = numpy.array([0, 0, 0])
                ray_dir_sensor = orientation_helpers.xyz_array(feedback.position)
                ray_dir_sensor = ray_dir_sensor / numpy.linalg.norm(ray_dir_sensor)
                
                ray_start_world = map_transform.transform_point(
                    transform.transform_point(ray_start_sensor))
                ray_dir_world = map_transform.transform_vector(
                    transform.transform_vector(ray_dir_sensor))
                
                movement_plane_world = numpy.array([0, 0, 1])
                
                plane_point = numpy.array([0, 0, bottom_z])
                plane_vector = numpy.array([0, 0, 1])
                
                x = plane_vector.dot(ray_start_world - plane_point) / plane_vector.dot(ray_dir_world)
                object_pos = ray_start_world - ray_dir_world * x
                print 'object_pos_body:', map_transform.inverse().transform_point(object_pos)
                
                desired_pos = object_pos - map_transform.transform_vector(transform.transform_point(ray_start_sensor))
                desired_pos = desired_pos - movement_plane_world * movement_plane_world.dot(desired_pos - start_pose.position)
                
                error_pos = desired_pos - map_transform.transform_point([0, 0, 0])
                error_pos = error_pos - movement_plane_world * movement_plane_world.dot(error_pos)
                
                threshold = 1 # m
                angle_error = abs(math.acos((error_pos / numpy.linalg.norm(error_pos)).dot(self.pose.forward_vector)))
                print 'pos error:', numpy.linalg.norm(error_pos), 'angle error:', math.degrees(angle_error), 'rel pos error:', numpy.linalg.norm(error_pos)/threshold
                
                if numpy.linalg.norm(error_pos) < threshold:
                    good.append(desired_pos)
                    if len(good) > 3:
                        yield (self.move
                            .set_orientation(orientation)
                            .set_position(numpy.mean(good, axis=0))
                            .go())
                        
                        return
                else:
                    good = []
                
                # go towards desired position
                if numpy.linalg.norm(error_pos) > 4:
                    move_goal_mgr = self._moveto_action_client.send_goal(
                        self.pose.look_at_without_pitching(desired_pos).set_position((desired_pos+self.pose.position)/2).as_MoveToGoal())
                    orientation = self.pose.look_at_without_pitching(desired_pos).orientation
                else:
                    move_goal_mgr = self._moveto_action_client.send_goal(
                        self.pose.set_orientation(orientation).set_position((desired_pos+self.pose.position)/2).as_MoveToGoal())
        finally:
            if move_goal_mgr is not None: yield move_goal_mgr.cancel()
    
    @util.cancellableInlineCallbacks
    def get_z_force(self):
        wrench = yield self._wrench_sub.get_next_message()
        defer.returnValue(wrench.wrench.force.z)


_subs = {}
@util.cancellableInlineCallbacks
def get_sub(node_handle, need_trajectory=True):
    if node_handle not in _subs:
        _subs[node_handle] = None # placeholder to prevent this from happening reentrantly
        _subs[node_handle] = yield _Sub(node_handle)._init(need_trajectory)
        # XXX remove on nodehandle shutdown
    defer.returnValue(_subs[node_handle])

def select_by_body_direction(body_vector):
    body_vector = numpy.array(body_vector)
    def _(results, body_tf):
        def get_wantedness(result):
            pos_vec = numpy.array(map(float, result['center']))
            pos_vec_body = body_tf.transform_vector(pos_vec)
            return pos_vec_body.dot(body_vector)

        return max(results, key=get_wantedness)
    return _
