#!/usr/bin/env python  
# -*- coding: utf-8 -*-
# HelloBot2.0 Base ROS Driver ver1.0
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

class Hellobot1_Base_Driver:

    const.Wheelbase=0.348                                                      #Wheel base
    const.EncoderResolution=1200                                               #Encoder ticks per revolution
    const.WheelDiameter=0.129                                                  #Wheel diameter
    const.pi=3.14159265358                                                    
    const.PowerRange=255                                                       #PWM power range

    UpdateWheelDirection      = '0200%s00%s'                                    
    UpdateWheelSpeed          = '0200%s00%s'                                  
    OdometryEnable            = '04f0'                                         
    OdometryDisable           = '040f'                                         
    DataInput=[]*10                                                                        

    x  = 0.0                                                                   
    y  = 0.0
    th = 0.0                                                             

    vx = 0.0
    vy = 0.0
    vth = 0.0

    clock_front = 0.0                                                            
    clock_back  = 0.0                                                                              

    
    #Converting speed into odom
    def speed_to_odom(self,Lspeed = 0, Rspeed = 0):     
        v_th = (Rspeed - Lspeed)/const.Wheelbase                              
        v_x  = (Rspeed + Lspeed)/2                                            
        return v_x, v_th  
    

    #Converting odom to speed
    def odom_to_speed(self,msg_linear_x =0, msg_linear_y=0, msg_angular_z=0):  
        self.yawrate2 = msg_angular_z*const.Wheelbase                        
        self.Lwheelspeed = msg_linear_x - self.yawrate2/2  
        self.Rwheelspeed = msg_linear_x + self.yawrate2/2 
        #rospy.loginfo("self.yawrate2=:%s",self.yawrate2)
        return self.Lwheelspeed, self.Rwheelspeed  


    #Sending speed command to HelloBot1 base
    def odom_analysis(self,msg):  
        (self.left_wheel_speed,self.right_wheel_speed)=self.odom_to_speed(msg.linear.x, msg.linear.y, msg.angular.z)
        self.left_encoder=((self.left_wheel_speed/10) / ((const.WheelDiameter*const.pi)/const.EncoderResolution))//1 
        self.right_encoder=((self.right_wheel_speed/10) / ((const.WheelDiameter*const.pi)/const.EncoderResolution))//1  
        self.left_encoderInt=int(self.left_encoder)              
        self.right_encoderInt=int(self.right_encoder)

        # Positive = front, Negative = back
        if self.left_encoderInt >=0:
            self.left_direction_flag = '0C'          
            self.left_direction      = '00'       
        else:
            self.left_direction_flag = '0C'
            self.left_direction      = '01'

        if self.right_encoderInt >=0:
            self.right_direction_flag = '0A'
            self.right_direction      = '00'        
        else:
            self.right_direction_flag = '0A'
            self.right_direction      = '01'
        
        self.Left_Direction_Send = Hellobot1_Base_Driver.UpdateWheelDirection%(self.left_direction_flag,self.left_direction)
        self.ser.write(self.Left_Direction_Send.decode('hex'))
        time.sleep(0.001)
        #rospy.loginfo("self.Left_Direction_Send=%s",self.Left_Direction_Send)

        self.Right_Direction_Send = Hellobot1_Base_Driver.UpdateWheelDirection%(self.right_direction_flag,self.right_direction)
        self.ser.write(self.Right_Direction_Send.decode('hex'))
        time.sleep(0.001)
        #rospy.loginfo("self.Right_Direction_Send=%s",self.Right_Direction_Send)

        if abs(self.left_encoderInt)>const.PowerRange:
            self.left_speed_flag  = '0D'
            self.left_speed=(hex(const.PowerRange))[2:]                          
        else:
            self.left_speed_flag  = '0D'
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
        
        self.Left_Speed_Send = Hellobot1_Base_Driver.UpdateWheelSpeed%(self.left_speed_flag,self.left_speed)
        self.ser.write(self.Left_Speed_Send.decode('hex'))
        time.sleep(0.001)
        #rospy.loginfo("self.Left_Speed_Send=%s",self.Left_Speed_Send)

        self.Right_Speed_Send = Hellobot1_Base_Driver.UpdateWheelSpeed%(self.right_speed_flag,self.right_speed)
        self.ser.write(self.Right_Speed_Send.decode('hex'))
        time.sleep(0.001)
        #rospy.loginfo("self.Right_Speed_Send=%s",self.Right_Speed_Send)

        self.ser.flush()

    #Read actual speed from STM32 and 
    def odom_synthesis(self,l_direction,l_speed,r_direction,r_speed):
        if l_direction==0:
            self.left_wheel_actual_speed=(l_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10)*-1
        else:
            self.left_wheel_actual_speed=l_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10
        if r_direction==0:
            self.right_wheel_actual_speed=(r_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10)*-1
        else:
            self.right_wheel_actual_speed=r_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10
        (self.linear_x,self.angular_z)=self.speed_to_odom(self.left_wheel_actual_speed, self.right_wheel_actual_speed)
        return self.linear_x,0.0,self.angular_z        

    def run(self):  
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        rospy.init_node('Hellobot2_Odometry_Publisher', anonymous=True)
        rospy.Subscriber("/planner/cmd_vel", Twist, self.odom_analysis)
        rospy.loginfo("HelloBot2.0 Connected Succesfully!")

        self.odomPub = rospy.Publisher('/vehicle/odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()                         		#Initialize TF broadcast
        self.ser.write(Hellobot1_Base_Driver.OdometryEnable.decode('hex'))   		#Enable data transfer

        Hellobot1_Base_Driver.clock_front=rospy.Time.now()
        Hellobot1_Base_Driver.clock_back=rospy.Time.now()

        delay = rospy.Rate(20)
        while not rospy.is_shutdown(): 
  
            #Receiving wheel direction and speed from STM32
            while self.ser.inWaiting() > 0:
                """Hellobot1_Base_Driver.DataInput=self.ser.read(6)
                if ord(Hellobot1_Base_Driver.DataInput[1]) == 0:
                    #rospy.loginfo("HelloBot1.0 Sending Data to ROS!")
                    (Hellobot1_Base_Driver.vx,Hellobot1_Base_Driver.vy,Hellobot1_Base_Driver.vth)=\
                    self.odom_synthesis(ord(Hellobot1_Base_Driver.DataInput[0]),ord(Hellobot1_Base_Driver.DataInput[2])\
                                       ,ord(Hellobot1_Base_Driver.DataInput[3]),ord(Hellobot1_Base_Driver.DataInput[5]))
                    
                    Hellobot1_Base_Driver.clock_front=rospy.Time.now()
                    self.dt=0.1
                    self.delta_x = (Hellobot1_Base_Driver.vx*cos(Hellobot1_Base_Driver.th) - Hellobot1_Base_Driver.vy*sin(Hellobot1_Base_Driver.th)) * self.dt 
                    self.delta_y = (Hellobot1_Base_Driver.vx*sin(Hellobot1_Base_Driver.th) + Hellobot1_Base_Driver.vy*cos(Hellobot1_Base_Driver.th)) * self.dt 
                    self.delta_th= Hellobot1_Base_Driver.vth * self.dt
        
                    Hellobot1_Base_Driver.x += self.delta_x
                    Hellobot1_Base_Driver.y += self.delta_y
                    Hellobot1_Base_Driver.th += self.delta_th
                    rospy.loginfo("self.ser.read(6)=:%s", self.ser.read(6).encode('hex'))
                    rospy.loginfo("Left_Wheel_Actual_Direction=:%s", ord(Hellobot1_Base_Driver.DataInput[0]))
                    rospy.loginfo("Left_Wheel_Actual_Speed=:%s", ord(Hellobot1_Base_Driver.DataInput[2]))
                    rospy.loginfo("Right_Wheel_Actual_Direction=:%s", ord(Hellobot1_Base_Driver.DataInput[3]))
                    rospy.loginfo("Right_Wheel_Actual_Speed=:%s", ord(Hellobot1_Base_Driver.DataInput[5]))
                    
                    self.quaternion = Quaternion()
                    self.quaternion.x = 0.0 
                    self.quaternion.y = 0.0
                    self.quaternion.z = sin(Hellobot1_Base_Driver.th / 2.0)
                    self.quaternion.w = cos(Hellobot1_Base_Driver.th / 2.0)
                    
                    self.odom_quat=tf.transformations.quaternion_from_euler(0, 0, Hellobot1_Base_Driver.th)
					
                    #发布tf广播
                    self.odomBroadcaster.sendTransform((Hellobot1_Base_Driver.x, Hellobot1_Base_Driver.y, 0)\
                                                       ,(self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w)\
                                                       ,Hellobot1_Base_Driver.clock_front\
                                                       ,"base_link"\
                                                       ,"odom"
                                                      )
                    #发布机器人本体消息
                    self.odom = Odometry()
                    self.odom.header.frame_id = "base_link"
                    self.odom.child_frame_id = "odom"
                    self.odom.header.stamp = Hellobot1_Base_Driver.clock_front
                    self.odom.pose.pose.position.x  = Hellobot1_Base_Driver.x
                    self.odom.pose.pose.position.y  = Hellobot1_Base_Driver.y
                    self.odom.pose.pose.position.z  = 0.0
                    self.odom.pose.pose.orientation = self.quaternion
                    self.odom.twist.twist.linear.x  = Hellobot1_Base_Driver.vx
                    self.odom.twist.twist.linear.y  = Hellobot1_Base_Driver.vy
                    self.odom.twist.twist.angular.z = Hellobot1_Base_Driver.vth
                    self.odomPub.publish(self.odom)
					"""
                Hellobot1_Base_Driver.DataInput=[]
                self.ser.flushInput()
                break

            rospy.Rate(20)

if __name__ == '__main__':  
    try:
        HelloBot1 = Hellobot1_Base_Driver()
        HelloBot1.run()  
    except rospy.ROSInterruptException: pass


























 
