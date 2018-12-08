#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
import time

def TwistStamped_To_Twist(sim_vel):
        robot_base_vel = Twist()
        robot_base_vel.linear.x = sim_vel.twist.linear.x
        robot_base_vel.linear.y = sim_vel.twist.linear.y
        robot_base_vel.linear.z = sim_vel.twist.linear.z
        robot_base_vel.angular.x = sim_vel.twist.angular.x
        robot_base_vel.angular.y = sim_vel.twist.angular.y 
        robot_base_vel.angular.z = sim_vel.twist.angular.z
        robot_base_vel_Pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)
        robot_base_vel_Pub.publish(robot_base_vel)
        	
def robot_base_vel_listener():
        rospy.Subscriber("/twist_cmd", TwistStamped, TwistStamped_To_Twist) #from Autoware OpenPlanner simulator
        rospy.spin()
        
if __name__ == '__main__':
	    rospy.init_node('robot_base_vel_listener', anonymous=True)
	    rospy.loginfo("TwistStamped To Twist Activated!")
	    robot_base_vel_listener()
