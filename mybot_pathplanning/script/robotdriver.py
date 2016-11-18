#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib;

roslib.load_manifest('mybot_pathplanning')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from python_serial import *
from  mybot_pathplanning.msg import Num  # beginner_tutorials 为自己建立的package,放在catkin_ws/src下
from  mybot_pathplanning.msg import carOdom  # beginner_tutorials 为自己建立的package,放在catkin_ws/src下
import os
import time
import binascii

thread = COMThread()  # serial


def callback(msg):
    # rospy.loginfo("transform /cmd_vel!")
    # rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    # rospy.loginfo("vx=%fm/s,wz=%frad/s"%(msg.linear.x,msg.angular.z))

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
    dist_bt_wheel = 0.202
    v_l = ((2 * msg.linear.x) - (msg.angular.z * dist_bt_wheel)) / 2
    v_r = ((2 * msg.linear.x) + (msg.angular.z * dist_bt_wheel)) / 2
    # rospy.loginfo("lwheel=%fm/s,rwheel=%fm/s"%(v_l,v_r))
    # v_l_str = str(v_l)
    # v_r_str = str(v_r)
    v_l_str = '%-+9.6f' % v_l
    v_r_str = '%-+9.6f' % v_r
    v_check_head = "U"
    v_combine = v_check_head + v_l_str + v_r_str
    # rospy.loginfo(v_l_str)
    # rospy.loginfo(v_combine)
    thread.ser.write(v_combine)  # serial send


# Then set your wheel speeds (using wheel_left and wheel_right as examples)
#    wheel_left.set_speed(v_l)  
#    wheel_right.set_speed(v_r)  

def talker():
    def speed_to_odom(Lspeed=0, Rspeed=0):  # Vr and Vl turn into Vx and Wz
        v_x = (Lspeed + Rspeed) / 2  # Vx
        v_y = 0
        v_th = (Rspeed - Lspeed) / 0.202  # Wz
        return v_x, v_y, v_th

    # os.system('echo 6 > /sys/class/gpio/export')
    # os.system('echo "out" > /sys/class/gpio/gpio6/direction')
    # os.system('echo 1 > /sys/class/gpio/gpio6/value')
    rospy.init_node('robotdriver', anonymous=True)
    thread.start()

    rospy.Subscriber("/cmd_vel", Twist, callback)  # subscribe cmd_vel topic
    pub_car_speed = rospy.Publisher('car_speed', carOdom, queue_size=1)  # real param turn into Vx,Wz
    pub_wheel_speed = rospy.Publisher('wheel_speed', Num, queue_size=1)  # real Vl,Vr

    r = rospy.Rate(50)  # 1000hz
    while not rospy.is_shutdown():
        # time.sleep (1)
        # all_data = []
        # if rec_data.com_isopen():
        # all_data = rec_data.next()  #接收的数据组
        # ser.write("0.7000000.700000")
        wheelspeed = Num()
        carspeed = carOdom()
        all_data = ""
        if event1.isSet():  # 如果event1为真，则没在更新
            # time.sleep(0.008)
            event2.clear()  # 设置event2为假，表示正在读取中
            # print "reading data"
            all_data = thread.all_data
            event2.set()  # 设置event2为真，表示读取完毕
            # print all_data
            # print "reading finished"
        if all_data.strip() != "":
            aa_location = all_data.find("aa")
            if aa_location != -1:
                # print "pub_start"
                # print all_data
                # print "after queue"
                data_l = all_data[aa_location + 2:aa_location + 2 + 16]
                data_r = all_data[aa_location + 2 + 16:aa_location + 2 + 32]
                data_th = all_data[aa_location + 2 + 32:aa_location + 2 + 44]
                data_w = all_data[aa_location + 2 + 44:aa_location + 2 + 56]
                # print data_l
                # print data_r
                # print data_th
                # print data_w
                data_l = binascii.a2b_hex(data_l)
                data_r = binascii.a2b_hex(data_r)
                data_th = binascii.a2b_hex(data_th)
                data_w = binascii.a2b_hex(data_w)
                t_data_l = float(data_l)
                t_data_r = float(data_r)
                t_data_th = float(data_th)
                t_data_w = float(data_w)
                # print(all_data)
                results = speed_to_odom(t_data_l, t_data_r)
                carspeed.x = results[0]
                carspeed.y = results[1]
                # carspeed.vth = results[2]#is going to be changed
                carspeed.vth = t_data_w * 0.017453
                carspeed.th = t_data_th * 0.017453
                wheelspeed.leftspeed = t_data_l
                wheelspeed.rightspeed = t_data_r
                # print carspeed
                # print wheelspeed
                # print "running pub"

                pub_car_speed.publish(carspeed)
                pub_wheel_speed.publish(wheelspeed)
            # if all_data != []:
            # car_speed = Num()                  #注意 消息的使用
            # car_speed.leftspeed = all_data[0][0]
            # car_speed.rightspeed = all_data[1][0]
            # print car_speed.leftspeed, car_speed.rightspeed
            # pub.publish(car_speed)
            else:
                print 'header error'
        r.sleep()
    thread.stop()
    # os.system('echo 0 > /sys/class/gpio/gpio6/value')
    # os.system('echo 6 > /sys/class/gpio/unexport')


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("end")
