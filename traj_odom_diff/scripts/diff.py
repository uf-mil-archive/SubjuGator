#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from uf_common.msg import PoseTwistStamped
from traj_odom_diff.msg import pose_diff

pub = rospy.Publisher("pose_diff", pose_diff) 
rospy.init_node("traj_listener", anonymous=True)
r = rospy.Rate(10) #10Hz

odomUpdated = 0
trajUpdated = 0
odom = pose_diff()
traj = pose_diff()

def publish():
	global odomUpdated
	global trajUpdated
	msg = pose_diff()
	msg.diff_x = traj.diff_x - odom.diff_x
	msg.diff_y = traj.diff_x - odom.diff_y
	msg.diff_z = traj.diff_x - odom.diff_z
	pub.publish(msg)
	odomUpdated = 0
	trajUpdated = 0

def callback(data):
	global odom
	global odomUpdated
	global trajUpdated
	odom.diff_x = data.pose.pose.position.x
	odom.diff_y = data.pose.pose.position.y
	odom.diff_z = data.pose.pose.position.z
	odomUpdated = 1
	if((odomUpdated == 1) and (trajUpdated == 1)):
		publish()

def trajCallback(data):
	global traj
	global odomUpdated
	global trajUpdated
	traj.diff_x = data.posetwist.pose.position.x
	traj.diff_y = data.posetwist.pose.position.y
	traj.diff_z = data.posetwist.pose.position.z
	trajUpdated = 1
	if(odomUpdated and trajUpdated):
		publish()

def listener():
	rospy.Subscriber("odom", Odometry, callback)
	rospy.Subscriber("trajectory", PoseTwistStamped, trajCallback)

	rospy.spin()

if __name__ == "__main__":
	listener()

