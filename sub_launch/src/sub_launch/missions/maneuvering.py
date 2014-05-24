from __future__ import division

import itertools
import time
import math

from geometry_msgs.msg import PoseWithCovariance, Quaternion, Pose, Point
from txros import util
import roslib

import sub_scripting
from object_finder import msg as object_finder_msg
from sim import threed

def from_obj(filename):
    mesh = threed.mesh_from_obj(filename)
    
    res = object_finder_msg.Mesh()
    
    components = []
    for mtl, x in itertools.groupby(zip(mesh.indices, mesh.materials), lambda (triangle, mtl): mtl):
        triangles = []
        for triangle, mtl_ in x:
            assert len(triangle) == 3
            assert mtl_ is mtl
            
            triangles.append(object_finder_msg.Triangle(
                corners=[Point(*mesh.vertices[vert_index]) for vert_index, tex_index, normal_index in triangle],
            ))
        
        components.append(object_finder_msg.Component(
            name=mtl.name,
            color=object_finder_msg.Color(
                r=mtl.Kd[0],
                g=mtl.Kd[1],
                b=mtl.Kd[2],
            ),
            triangles=triangles,
        ))
    
    components.sort(key=lambda component: component.name)
    
    return object_finder_msg.Mesh(components=components)

maneuvering_targetdesc = object_finder_msg.TargetDesc(
    type=object_finder_msg.TargetDesc.TYPE_MESH,
    mesh=from_obj(roslib.packages.resource_file('auvsi_robosub', 'models', '2014/maneuvering_vision.obj')),
    prior_distribution=PoseWithCovariance(
        pose=Pose(
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        ),
        covariance=[
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
        ],
    ),
    min_dist=0,
    max_dist=5,
)

@util.cancellableInlineCallbacks
def main(nh):
    print 1
    
    sub = yield sub_scripting.get_sub(nh)
    
    print 2
    
    yield sub.visual_approach_3d('forward', 2.5, maneuvering_targetdesc)
    
    r = 1
    
    start_cmd = sub.move
    
    yield start_cmd.turn_right_deg(90).go()
    
    try:
        yield sub.set_trajectory_generator_enable(False)
        
        start = time.time()
        
        strafe_vel = 0.3
        total_time = 2*math.pi*r / strafe_vel
        angvel = 2*math.pi / total_time
        while True:
            t = time.time() - start
            
            x = t/total_time
            
            if x >= 1.25:
                break
            
            ang = x * 2*math.pi
            
            (start_cmd.forward(r)
                .relative((-r*math.cos(ang), -r*math.sin(ang), 0))
                .yaw_left(ang)
                .turn_right_deg(90)
                .go_trajectory(
                    linear=[strafe_vel, 0, 0],
                    angular=[0, 0, angvel],
                ))
            
            yield util.sleep(0.01)
        
    finally:
        yield sub.set_trajectory_generator_enable(True)
    
    print 10
    yield start_cmd.forward(r).right(r).forward(2).go()
    print 11
