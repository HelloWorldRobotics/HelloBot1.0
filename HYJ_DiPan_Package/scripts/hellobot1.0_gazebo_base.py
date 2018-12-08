#!/usr/bin/env python  
# -*- coding: utf-8 -*-
                                                                                              
import time
import serial
import const                                                                  #常量定义类
import roslib; roslib.load_manifest('HYJ_DiPan_Package')  
import rospy  
import tf.transformations  

from math import sin, cos, pi
from geometry_msgs.msg import Twist  
#from hyj_odom_tf_package.msg import Num                                       #自己定义的消息类型
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion, Twist, Pose

rospy.init_node('hellobot_gazebo_base_odom_pub')

odom_pub=rospy.Publisher ('/vehicle/odom', Odometry, queue_size=5)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='model'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    r.sleep()