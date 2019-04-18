#!/usr/bin/env python  
# -*- coding: utf-8 -*-
# HelloBot2.0 Base ROS Driver ver1.0
"""
这个在unix类的操作系统才有意义。
#!/usr/bin/python是告诉操作系统执行这个脚本的时候，调用/usr/bin下的python解释器；
#!/usr/bin/env python这种用法是为了防止操作系统用户没有将python装在默认的/usr/bin路径里。
当系统看到这一行的时候，首先会到env设置里查找python的安装路径，再调用对应路径下的解释器程序完成操作。
在windows中设置了环境变量后可以直接“hello.py”

或者这样解释：
加上
#!/usr/bin/env python, 这个py就处于了可执行模式下, (当然是针对linux类的操作系统),  这个hint, 
告诉操作系统要使用哪个python解释器来执行这个py. 在linux上执行一下命令 /usr/bin/env python ,
就知道这行其实是call一下python解释器.  这种写法比#! /usr/bin/python要好, 后者是hard coding 了python的路径.
"""
import sys
import time
import serial
import const
import roslib
import rospy  
import tf.transformations  

from math import sin, cos, pi
from geometry_msgs.msg import Twist  
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion, Twist, Pose
from functools import reduce

class HelloBot2_Base_Driver:

    #Wheel Speed = PWM*factor + offset
    const.factor_left	    = 0.0245
    const.offset_left	    = 0.0019
    const.factor_right	    = 0.0246
    const.offset_right	    = 0.0006
    const.Wheelbase         = 0.50                                                   #Wheel base
    const.EncoderResolution = 400                                                    #Encoder ticks per revolution
    const.WheelDiameter     = 0.22                                                   #Wheel diameter
    const.pi                = 3.14159265358                                                    
    const.PowerRange        = 255                                                    #PWM power range

    UpdateWheelDirection    = '0200%s00%s'                                    
    UpdateWheelSpeed        = '0200%s00%s'                                  
    OdometryEnable          = '04f0'                                         
    OdometryDisable         = '040f'                                         
    DataInput=[]*10                                                                 

    left_wheel_velocity	    = 0.0
    right_wheel_velocity    = 0.0
    x  = 0.0
    y  = 0.0
    th = 0.0

    def is_number(self,n):
    	is_number = True
    	try:
            num = float(n)
            is_number = num == num   #or using math.isnan(num)
    	except ValueError:
       	    is_number = False
	    print(is_number)
	return is_number

    #Converting velocity into odom
    def velocity_to_odom(self,left_wheel_actual_speed,right_wheel_actual_speed):     
        v_th = (right_wheel_actual_speed - left_wheel_actual_speed)/const.Wheelbase                              
        v_x  = (right_wheel_actual_speed + left_wheel_actual_speed)/2
        v_y  = 0.0                        
        return v_x,v_y,v_th  
    
    #Converting odom to speed
    def odom_to_speed(self,msg_linear_x, msg_linear_y, msg_angular_z):  
        self.yawrate2 = msg_angular_z*const.Wheelbase                        
        self.Lwheelspeed = msg_linear_x - self.yawrate2/2  
        self.Rwheelspeed = msg_linear_x + self.yawrate2/2
        return self.Lwheelspeed, self.Rwheelspeed  

    #Sending speed command to HelloBot2 base
    #xxxxWheelV = PWM *factor + offset
    def velocity_callback(self,msg):  
        (self.left_wheel_speed,self.right_wheel_speed)=self.odom_to_speed(msg.linear.x, msg.linear.y, msg.angular.z)
	#Wheel Speed = PWM*factor + offset
        self.left_encoder	= ((self.left_wheel_speed-const.offset_left)/const.factor_left)
        self.right_encoder	= ((self.right_wheel_speed-const.offset_right)/const.factor_right)
        self.left_encoderInt	= int(self.left_encoder)             
        self.right_encoderInt	= int(self.right_encoder)
	#print(self.left_encoderInt,self.right_encoderInt) 
        # Positive = front, Negative = back
        if self.left_encoderInt >=0:
            self.left_direction_flag  = '0C'          
            self.left_direction       = '00'       
        else:
            self.left_direction_flag  = '0C'
            self.left_direction       = '01'

        if self.right_encoderInt >=0:
            self.right_direction_flag = '0A'
            self.right_direction      = '00'        
        else:
            self.right_direction_flag = '0A'
            self.right_direction      = '01'
        
        self.Left_Direction_Send = HelloBot2_Base_Driver.UpdateWheelDirection%(self.left_direction_flag,self.left_direction)
        self.ser.write(self.Left_Direction_Send.decode('hex'))
        time.sleep(0.01)

        self.Right_Direction_Send = HelloBot2_Base_Driver.UpdateWheelDirection%(self.right_direction_flag,self.right_direction)
        self.ser.write(self.Right_Direction_Send.decode('hex'))
        time.sleep(0.01)

        if abs(self.left_encoderInt)>const.PowerRange:
            self.left_speed_flag   = '0D'
            self.left_speed=(hex(const.PowerRange))[2:]                          
        else:
            self.left_speed_flag   = '0D'
            self.left_speed=(hex(abs(self.left_encoderInt)))[2:]
            if len(self.left_speed)==1:
                self.left_speed='0'+self.left_speed
        if abs(self.right_encoderInt)>const.PowerRange:
            self.right_speed_flag  = '0B'
            self.right_speed=(hex(const.PowerRange))[2:]                           
        else:
            self.right_speed_flag  = '0B'
            self.right_speed=(hex(abs(self.right_encoderInt)))[2:]    
            if len(self.right_speed)==1:
                self.right_speed='0'+self.right_speed
        
        self.Left_Speed_Send = HelloBot2_Base_Driver.UpdateWheelSpeed%(self.left_speed_flag,self.left_speed)
        self.ser.write(self.Left_Speed_Send.decode('hex'))
        time.sleep(0.01)

        self.Right_Speed_Send = HelloBot2_Base_Driver.UpdateWheelSpeed%(self.right_speed_flag,self.right_speed)
        self.ser.write(self.Right_Speed_Send.decode('hex'))
        time.sleep(0.01)

        self.ser.flush()

    def run(self):
        rate = rospy.Rate(20)
        self.ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.1)
        rospy.Subscriber("/planner/cmd_vel",Twist,self.velocity_callback)
        rospy.loginfo("HelloBot2.0 Connected Succesfully!")

        self.odomPub = rospy.Publisher('/vehicle/odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()                         		#Initialize TF broadcast
        self.ser.write(HelloBot2_Base_Driver.OdometryEnable.decode('hex'))   		#Enable data transfer
        time.sleep(0.01)
	
        current_time = rospy.Time.now()
        eplased_time = rospy.Time.now()

        while not rospy.is_shutdown():
            #Receiving wheel direction and speed from STM32
	    """data = ''
	    data = data.encode('utf-8')
	    n = self.ser.inWaiting()
	
	    if n:
		data = data + self.ser.readline()
		print("serial data->", data)
		print(type(data))"""

            while self.ser.inWaiting() > 0:#返回接收缓存中的字节数
		#making sure the values are float numbers
		try:
			HelloBot2_Base_Driver.DataInput = self.ser.readline().split('?')
			v_left_wheel   = float(HelloBot2_Base_Driver.DataInput[1])
			v_right_wheel  = float(HelloBot2_Base_Driver.DataInput[2])
		except ValueError:
			print("NOT A FLOAT, PASS->")
			break
		except IndexError:
			print("LIST INDEX ERROR, PASS->")
			break

		#print(isinstance(v_left_wheel, float))
		#print(v_left_wheel,v_right_wheel)
		#print(self.is_number(v_left_wheel),self.is_number(v_right_wheel))
                (vx,vy,vth) = self.velocity_to_odom(v_left_wheel,v_right_wheel)
		#(vx,vy,vth) = self.velocity_to_odom(0,0)
                current_time = rospy.Time.now()
                dt  = (current_time - eplased_time).to_sec()
                dx  = (vx*cos(HelloBot2_Base_Driver.th) - vy*sin(HelloBot2_Base_Driver.th)) * dt
                dy  = (vx*sin(HelloBot2_Base_Driver.th) + vy*cos(HelloBot2_Base_Driver.th)) * dt
                dth = (vth)*dt

                HelloBot2_Base_Driver.x  += dx
                HelloBot2_Base_Driver.y  += dy
                HelloBot2_Base_Driver.th += dth

                self.odom = Odometry()
                self.odom.header.frame_id = "odom"
                self.odom.child_frame_id  = "base_link"
                self.odom.header.stamp = current_time
                self.odom.pose.pose.position.x = HelloBot2_Base_Driver.x
                self.odom.pose.pose.position.y = HelloBot2_Base_Driver.y
                self.odom.pose.pose.position.z = 0.0
            
                q = tf.transformations.quaternion_from_euler(0,0,HelloBot2_Base_Driver.th)
                self.odom.pose.pose.orientation.x = q[0]
                self.odom.pose.pose.orientation.y = q[1]
                self.odom.pose.pose.orientation.z = q[2]
                self.odom.pose.pose.orientation.w = q[3]
                self.odom.twist.twist.linear.x  = vx
                self.odom.twist.twist.linear.y  = vy
                self.odom.twist.twist.angular.z = vth
                self.odomPub.publish(self.odom)

                eplased_time = current_time
                self.ser.flushInput()
                rate.sleep()

if __name__ == '__main__':  
    try:
        rospy.init_node('Hellobot2_Odometry_Publisher',anonymous=True)
        HelloBot2 = HelloBot2_Base_Driver()
        HelloBot2.run()  
    except rospy.ROSInterruptException: pass

