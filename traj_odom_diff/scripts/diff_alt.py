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

def callback(data):
	odomUpdated = 1
	odom = pose_diff()
	odom.diff_x = data.pose.pose.position.x
	odom.diff_y = data.pose.pose.position.y
	odom.diff_z = data.pose.pose.position.z
	#if(odomUpdated && trajUpdated)
	#	publishDifference()

	msg = pose_diff()
	msg.diff_x = data.pose.pose.position.x
	msg.diff_y = data.pose.pose.position.y
	msg.diff_z = data.pose.pose.position.z
	rospy.loginfo("echo")	
	pub.publish(odom)
	#r.sleep()

def trajCallback(data):
	rospy.loginfo(data.posetwist.pose.position)
	msg = pose_diff()
	msg.diff_x = data.posetwist.pose.position.x
	msg.diff_y = data.posetwist.pose.position.y
	msg.diff_z = data.posetwist.pose.position.z
	pub.publish(msg)
	#r.sleep()

def publishDifference():
	msg = pose_diff()
	msg.diff_x = traj.diff_x - odom.diff_x
	msg.diff_y = traj.diff_y - odom.diff_y
	msg.diff_z = traj.diff_z - odom.diff_z 	

def listener():
	rospy.Subscriber("odom", Odometry, callback)
	#rospy.Subscriber("sim_odom", Odometry, callback)
	#rospy.Subscriber("trajectory", PoseTwistStamped, trajCallback)

	rospy.spin()

if __name__ == "__main__":
	listener()
