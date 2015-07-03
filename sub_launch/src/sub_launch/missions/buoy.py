from __future__ import division

from geometry_msgs.msg import PoseWithCovariance, Quaternion, Pose
from txros import util

import sub_scripting
from object_finder import msg as object_finder_msg


buoy_targetdesc = object_finder_msg.TargetDesc(
    type=object_finder_msg.TargetDesc.TYPE_SPHERE,
    sphere_radius=4.5 * 0.0254, # = 4.5 inch
    prior_distribution=PoseWithCovariance(
        pose=Pose(
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        ),
    ),
    min_dist=1,
    max_dist=8,
    sphere_color=object_finder_msg.Color(r=1, g=0, b=0),
    sphere_background_color=object_finder_msg.Color(r=0, g=1, b=1),
)

@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    orig_depth = -sub.pose.position[2]
    
    yield sub.visual_approach_3d('forward', 1.5, buoy_targetdesc)
    
    yield sub.move.forward(1.5).go()
    yield sub.move.forward(-1.5).go()
    yield sub.move.forward(4.5).depth(1).go()
