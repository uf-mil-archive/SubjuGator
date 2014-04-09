from __future__ import division

from twisted.internet import defer

from txros import action, util

from uf_common.msg import MoveToAction, MoveToGoal, PoseTwistStamped
from uf_common import orientation_helpers


class _MoveProxy(object):
    def __init__(self, sub):
        self._sub = sub
    
    def __getattr__(self, name):
        def _(*args, **kwargs):
            new_pose = getattr(self._sub._pose, name)(*args, **kwargs)
            self._sub._pose = new_pose # XXX fix this if cancellation support is later added
            return self._sub._moveto_actionclient.send_goal(
                new_pose.as_MoveToGoal()).get_result()
        return _

class _Sub(object):
    def __init__(self, node_handle):
        self._node_handle = node_handle
    
    @util.cancellableInlineCallbacks
    def _init(self):
        self._trajectory_sub = self._node_handle.subscribe('trajectory', PoseTwistStamped)
        self._moveto_actionclient = action.ActionClient(self._node_handle, 'moveto', MoveToAction)
        
        self._pose = orientation_helpers.PoseEditor.from_PoseTwistStamped(
            (yield self._trajectory_sub.get_next_message()))
        
        defer.returnValue(self)
    
    @property
    def move(self):
        return _MoveProxy(self)

_subs = {}
def get_sub(node_handle):
    if node_handle not in _subs:
        _subs[node_handle] = _Sub(node_handle)._init()
        # XXX remove on nodehandle shutdown
    return _subs[node_handle]
