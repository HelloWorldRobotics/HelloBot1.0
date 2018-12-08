#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf
import time

def set_initial_pose_for_glaobal_path():
	path_initial_pose = PoseWithCovarianceStamped()
	path_initial_pose.header.stamp = rospy.Time.now()
	path_initial_pose.header.frame_id = "world"
	path_initial_pose.pose.pose.position.x = 5.69165658951
	path_initial_pose.pose.pose.position.y = -4.81000471115
	path_initial_pose.pose.pose.position.z = 0.0
	#tf.Quaternion quat;
	#quat.setRPY(0,0,-0.659648559409)
	#tf.quaternionTFToMsg(quat,pose.pose.pose.orientation);
	path_initial_pose.pose.pose.orientation.x = 0.0
	path_initial_pose.pose.pose.orientation.y = 0.0
	path_initial_pose.pose.pose.orientation.z = -0.659648559409
	path_initial_pose.pose.pose.orientation.w = 0.751574199976
	path_initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	path_initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
	path_initial_pose_pub.publish(path_initial_pose)
	
def set_goal_pose_for_global_path():
	path_goal_pose = PoseStamped()
	path_goal_pose.header.stamp = rospy.Time.now()
	path_goal_pose.header.frame_id = "world"
	path_goal_pose.pose.position.x = 24.3183002472
	path_goal_pose.pose.position.y = -121.106773376
	path_goal_pose.pose.position.z = 0.0
	#tf.Quaternion quat;
	#quat.setRPY(0,0,0.873264431835)
	#tf.quaternionTFToMsg(quat,pose.pose.pose.orientation);
	path_goal_pose.pose.orientation.x = 0.0 
	path_goal_pose.pose.orientation.y = 0.0
	path_goal_pose.pose.orientation.z = 0.873264431835
	path_goal_pose.pose.orientation.w = 0.487246582432
	path_goal_pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
	path_goal_pose_pub.publish(path_goal_pose)
	
if __name__ == '__main__':
	rospy.init_node('set_initial_pose_goal_for_global_path', anonymous=True)
	#rospy.init_node('set_goal_pose_for_global_path', anonymous=True)
	set_initial_pose_for_glaobal_path()
	set_goal_pose_for_global_path()	
