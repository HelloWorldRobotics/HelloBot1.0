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
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion, Twist, Pose

class bluetooth_cmd:

    const.Wheelbase=0.348                                                     #左右轮距单位(米) 0.348
    const.EncoderResolution=1200                                              #编码器分辨率一圈=1200针
    const.WheelDiameter=0.129                                                 #轮子直径单位(米) 0.1236
    const.pi=3.14159265358                                                    #圆周率π
    const.PowerRange=255                                                      #功率范围

    UpdateWheelSpeed          ='00010000200001%s00%s%s00%s'                   #更新轮子速度CAN总线指令  
    WheelSpeedEnableEnable    ='00010000020001000000000000'                   #轮速接收使打开CAN总线指令
    WheelSpeedReceptionEnable ='00010000030001000000000000'                   #轮速接收使能关CAN总线指令
    DataIntput=[]*10                                                          #轮子实际速度接收
    BluetoothEnable=0                                                         #蓝牙遥控使能
    RemoteCommand=''                                                          #蓝牙遥控指令

    x = 0.0                                                                   
    y = 0.0
    th = 0.0                                                             

    vx=0.0
    vy=0.0
    vth=0.0

    clock_front=0.0                                                            #系统时间前
    clock_back=0.0                                                             #系统时间后                   

    
    #速度转换里程计(输入单位为M/s，输出单位为M/s)
    def speed_to_odom(self,Lspeed = 0, Rspeed = 0):     
        v_th = (Rspeed - Lspeed)/const.Wheelbase                              # 角速度=左右轮速的差乘以轮距
        v_x  = (Rspeed + Lspeed)/2                                            # 线速度=左右轮速的和除以二
        return v_x, v_th  
    

    #里程计转换速度 （输入单位为m/s,输出单位为m/s）
    def odom_to_speed(self,msg_linear_x =0, msg_linear_y=0, msg_angular_z=0):  
        self.yawrate2 = msg_angular_z*const.Wheelbase                         #左右轮差速=角速度/左右轮距
        self.Lwheelspeed = msg_linear_x - self.yawrate2/2  
        self.Rwheelspeed = msg_linear_x + self.yawrate2/2 
        #rospy.loginfo("self.yawrate2=:%s",self.yawrate2)
        return self.Lwheelspeed, self.Rwheelspeed  


    #里程计解析，将收到的里程计转化为左右两轮的速度发给底盘
    def odom_analysis(self,msg):  
        (self.ZuoLunSu,self.YouLunSu)=self.odom_to_speed(msg.linear.x, msg.linear.y, msg.angular.z)
                                                                              #将速度m/s转化为 整数编码器针数/0.1s
        self.ZuoLunZhenShu=((self.ZuoLunSu/10) / ((const.WheelDiameter*const.pi)/const.EncoderResolution))//1 
        self.YuoLunZhenShu=((self.YouLunSu/10) / ((const.WheelDiameter*const.pi)/const.EncoderResolution))//1  
        self.ZuoLunZhenShuInt=int(self.ZuoLunZhenShu)                         #转化为整数的int类型
        self.YuoLunZhenShuInt=int(self.YuoLunZhenShu)   


        if self.ZuoLunZhenShuInt >=0:                                         #方向判断
            self.ZuoFangXiang='01'       
        else:
            self.ZuoFangXiang='00'        
        if self.YuoLunZhenShuInt>=0:
            self.YuoFangXiang='01'        
        else:
            self.YuoFangXiang='00'        


        if abs(self.ZuoLunZhenShuInt)>const.PowerRange:                       #速度计算
            self.ZuoSuDu=(hex(const.PowerRange))[2:]                          
        else:
            self.ZuoSuDu=(hex(abs(self.ZuoLunZhenShuInt)))[2:]    
            if len(self.ZuoSuDu)==1:
                self.ZuoSuDu='0'+self.ZuoSuDu
        if abs(self.YuoLunZhenShuInt)>const.PowerRange:
            self.YuoSuDu=(hex(const.PowerRange))[2:]                           
        else:
            self.YuoSuDu=(hex(abs(self.YuoLunZhenShuInt)))[2:]    
            if len(self.YuoSuDu)==1:
                self.YuoSuDu='0'+self.YuoSuDu

        self.WheelSpeedEmission=bluetooth_cmd.UpdateWheelSpeed%(self.ZuoFangXiang,self.ZuoSuDu,self.YuoFangXiang,self.YuoSuDu)
        self.ser.write(self.WheelSpeedEmission.decode('hex'))

        bluetooth_cmd.BluetoothEnable=1                                       #蓝牙遥控使能,当执行过自主运行后则不能接收蓝牙遥控命令
        self.ser.flush()                                                      #刷新缓存

    #里程计合成，将底盘传送上来的左右轮实际速度合成里程计数据格式发布主题
    def odom_synthesis(self,l_direction,l_speed,r_direction,r_speed):
        if l_direction==0:
            self.ZuoLunShiJiSuDu=(l_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10)*-1
        else:
            self.ZuoLunShiJiSuDu=l_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10
        if r_direction==0:
            self.YuoLunShiJiSuDu=(r_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10)*-1
        else:
            self.YuoLunShiJiSuDu=r_speed*((const.WheelDiameter*const.pi)/const.EncoderResolution)*10
                                                                              #转化为线速度和角速度
        (self.linear_x,self.angular_z)=self.speed_to_odom(self.ZuoLunShiJiSuDu, self.YuoLunShiJiSuDu)

        '''
        wheelspeed = Num()                                                    #消息发布测试
        wheelspeed.linear_x =self.linear_x
        wheelspeed.angular_z=self.angular_z
        self.pub_wheel.publish(wheelspeed)                                    #发布消息 线速度和角速度
        '''
        return self.linear_x,0.0,self.angular_z        

    def main_program(self):  
    
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)       		#初始化底盘串口

        rospy.init_node('odometry_publisher', anonymous=True)                 		#初始化节点   
        rospy.Subscriber("/planner/cmd_vel", Twist, self.odom_analysis)       		#订阅cmd_vel主题有新内容时调用odom_analysis函数 
																					#若使用了速度平滑包则改为"/smoother_cmd_vel"主题名称
        self.odomPub = rospy.Publisher('robot_base_odom', Odometry, queue_size=5)   #初始化里程计消息发布
        self.odomBroadcaster = TransformBroadcaster()                         		#初始化tf广播
        #self.pub_wheel = rospy.Publisher('wheel_speed', Num)                		#测试发布消息初始化左右轮速度 
        self.ser.write(bluetooth_cmd. WheelSpeedEnableEnable.decode('hex'))   		#开启底盘轮速接收

        bluetooth_cmd.clock_front=rospy.Time.now()                            		#系统时钟记录初始化        
        bluetooth_cmd.clock_back=rospy.Time.now()

        delay=rospy.Rate(20)                                                  		#20HZ
        while not rospy.is_shutdown(): 
  
            #判断底盘是否上传轮子速度信息上来
            while self.ser.inWaiting() > 0:
                bluetooth_cmd.DataIntput=self.ser.read(10)
                if ord(bluetooth_cmd.DataIntput[0]) == 2:                     		#判断是不是回馈里程计信息
                    #rospy.loginfo("b=:%s",ord(bluetooth_cmd.DataIntput[6]))
                    #将收到的底盘实际左右轮速度进行处理合成里程计
                    (bluetooth_cmd.vx,bluetooth_cmd.vy,bluetooth_cmd.vth)=\
                    self.odom_synthesis(ord(bluetooth_cmd.DataIntput[4]),ord(bluetooth_cmd.DataIntput[6])\
                                       ,ord(bluetooth_cmd.DataIntput[7]),ord(bluetooth_cmd.DataIntput[9]))
                    
                    bluetooth_cmd.clock_front=rospy.Time.now()
                    self.dt=0.1
                    self.delta_x = (bluetooth_cmd.vx*cos(bluetooth_cmd.th) - bluetooth_cmd.vy*sin(bluetooth_cmd.th)) * self.dt 
                    self.delta_y = (bluetooth_cmd.vx*sin(bluetooth_cmd.th) + bluetooth_cmd.vy*cos(bluetooth_cmd.th)) * self.dt 
                    self.delta_th= bluetooth_cmd.vth * self.dt
        
                    bluetooth_cmd.x += self.delta_x
                    bluetooth_cmd.y += self.delta_y
                    bluetooth_cmd.th += self.delta_th
                    #rospy.loginfo("bluetooth_cmd.x=:%s",bluetooth_cmd.x)


                    
                    self.quaternion = Quaternion()
                    self.quaternion.x = 0.0 
                    self.quaternion.y = 0.0
                    self.quaternion.z = sin(bluetooth_cmd.th / 2.0)
                    self.quaternion.w = cos(bluetooth_cmd.th / 2.0)
                    
                    self.odom_quat=tf.transformations.quaternion_from_euler(0, 0, bluetooth_cmd.th)
					
                    #发布tf广播
                    self.odomBroadcaster.sendTransform((bluetooth_cmd.x, bluetooth_cmd.y, 0)\
                                                       ,(self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w)\
                                                       ,bluetooth_cmd.clock_front\
                                                       ,"base_link"\
                                                       ,"odom"
                                                      )
                    #发布机器人本体消息
                    self.odom = Odometry()
                    self.odom.header.frame_id = "odom"
                    self.odom.child_frame_id = "base_link"
                    self.odom.header.stamp = bluetooth_cmd.clock_front
                    self.odom.pose.pose.position.x = bluetooth_cmd.x
                    self.odom.pose.pose.position.y = bluetooth_cmd.y
                    self.odom.pose.pose.position.z = 0.0
                    self.odom.pose.pose.orientation =self.quaternion
                    self.odom.twist.twist.linear.x = bluetooth_cmd.vx
                    self.odom.twist.twist.linear.y = bluetooth_cmd.vy
                    self.odom.twist.twist.angular.z =bluetooth_cmd.vth
                    self.odomPub.publish(self.odom)
					
                bluetooth_cmd.DataIntput=[]                                   #清零
                self.ser.flushInput()                                         #刷新缓存
                break
       
            delay.sleep()


if __name__ == '__main__':  
    try:
        DuiXiang=bluetooth_cmd()
        DuiXiang.main_program()  
    except rospy.ROSInterruptException: pass


























 
