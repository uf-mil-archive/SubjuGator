from __future__ import division

import itertools

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
    
    print [component.name for component in components]
    
    return object_finder_msg.Mesh(components=components)

peg_targetdesc = object_finder_msg.TargetDesc(
    type=object_finder_msg.TargetDesc.TYPE_MESH,
    mesh=from_obj(roslib.packages.resource_file('auvsi_robosub', 'models', '2014/PowerPeg_vision.obj')),
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
            0, 0, 0, 0, 0, 1,
        ],
    ),
    min_dist=1,
    max_dist=3,
)

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    peg_targetdesc.prior_distribution.pose.orientation = Quaternion(*sub.pose.turn_left_deg(180).orientation)
    
    yield sub.visual_approach_3d('forward', 1.5, peg_targetdesc)
    
    #yield sub.move.forward(1.5).go()
    #yield sub.move.backward(1.5).go()
    
    #yield sub.move.depth(0.5).go()
