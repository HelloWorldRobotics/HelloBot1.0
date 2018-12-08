如何使用底盘？
====

#1)安装底盘ROS驱动包
git clone ...

#2)底盘串口别名绑定

-插上底盘USB转串口模块到电脑到USB口 执行
sudo sh HYJ_DiPan_COM_Name.sh
chmod +x HYJ_DiPan_COM_Name.sh
-python脚本
chmod +
-然后拔掉再重新插入 进行底盘串口别名绑定

OR

-设置UDEV
udevadm info -a -n ttyUSB0
cd /etc/udev/rules.d
sudo vim xxxx.rules
eg. KERNEL=="ttyUSB*", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", ATTRS{devpath}=="3", SYMLINK+="HYJ_DiPan"

#3)安装调试小工具(图形化)	

-rbx1功能包
git clone https://github.com/pirobot/rbx1.git
cd rbx1
git checkout indigo-devel
cd ~/catkin_ws
catkin_make
-安装simulator
sudo apt-get install ros-indigo-arbotix-*

#4)进行底盘驱动测试

-插上底盘USB转串口到电脑USB口，点击小工具界面的小红点，用鼠标拖动小红点进行底盘测试，可以看到轮子动起来了说明底盘驱动没问题了
rosrun HYJ_DiPan_Package hyj_tf_broadcaster_node.py
arbotix_gui

#5)下面进行角速度与线速度测试（本底盘经过测量轮距为34.8cm）

-先进行线速度校准测试
rosrun HYJ_DiPan_Package hyj_tf_broadcaster_node.py		
rosrun rbx1_nav calibrate_linear.py
rosrun rqt_reconfigure rqt_reconfigure
-通过修改～/catkin_ws/src/HYJ_DiPan_Package/nodes/hyj_tf_broadcaster_node.py
的第22行const.WheelDiameter=0.1236  #轮子直径单位(米) 0.1236
-再进行线速度校准

-进行角速度校准
rosrun rbx1_nav calibrate_angular.py
rosrun rqt_reconfigure rqt_reconfigure
通过修改～/catkin_ws/src/HYJ_DiPan_Package/nodes/hyj_tf_broadcaster_node.py
的第20行const.Wheelbase=0.348      #左右轮距单位(米) 0.348
再进行角速度校准
