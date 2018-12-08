#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time

def set_initial_pose_for_wf_simulator():
	path_initial_pose = PoseWithCovarianceStamped()
	path_initial_pose.header.stamp = rospy.Time.now() 
	path_initial_pose.header.frame_id = "world"
	path_initial_pose.pose.pose.position.x = 5.69165658951
	path_initial_pose.pose.pose.position.y = -4.81000471115
	path_initial_pose.pose.pose.position.z = 0.0
	path_initial_pose.pose.pose.orientation.x = 0.0
	path_initial_pose.pose.pose.orientation.y = 0.0
	path_initial_pose.pose.pose.orientation.z = -0.659648559409
	path_initial_pose.pose.pose.orientation.w = 0.751574199976
	path_initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	path_initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
	path_initial_pose_pub.publish(path_initial_pose)
	
if __name__ == '__main__':
	rospy.init_node('set_initial_pose_for_wf_simulator', anonymous=True)
	set_initial_pose_for_wf_simulator()
	rospy.spin()
