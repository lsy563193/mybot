#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib;roslib.load_manifest('mybot_pathplanning')
import rospy
import serial
import threading 
import time
from  mybot.msg import Num       #mybot 为自己建立的package,放在catkin_ws/src下
import Queue

#import serial_lisenning as COM_ctr            #串口模块，已经封装好，模块里是开线程不断读取串口
import glob
import binascii

event1 = threading.Event()#更新串口信号量
event2 = threading.Event()#读取all_data信号量

event1.set()
event2.set()

class COMThread(threading.Thread):
    def __init__(self,name = 'COMThread'):
        threading.Thread.__init__(self,name = name)
        self.ser = serial.Serial()
	self.allport = glob.glob('/dev/ttyUSB1')
	self.port = self.allport[0]#0:非阻塞，None:阻塞，x:等待x秒退出
	self.baud = 115200

	self.ser.setPort(self.port)
	self.ser.setBaudrate(self.baud)
	self.ser.setTimeout(0)#0:非阻塞，None:阻塞，x:等待x秒退出
        self.ser.open()
        self.all_data=self.ser.read(34)
        self.test_data = binascii.b2a_hex(self.all_data)
        self.thread_stop = False

 
        if self.ser is None:
            print '无法打开串口'
        else:
            print(self.ser)
        self.parser = None
    def run(self):
	while not self.thread_stop:
	    #print("aaaaaa")
	    #print "%s starts"%(self.getName())
            time.sleep(0.02)
            if self.thread_stop:
                print "break!"
                break
            if not event2.isSet():#若event2为假，即正在读取中，则不更新，一直等待到event2为真，然后开始更新
                print "thread update waiting"
                event2.wait()
                print "wait over"
            if self.thread_stop:
                print "break!"
                break            
            event1.clear()#更新中，设event1为假
            #print "thread updating serial"
	    #self.all_data = self.ser.read(34)#read 18 bits data from serial
            self.all_data = self.ser.read(58)#read 18 bits data from serial
            #print ("thread_update_data=%s"%self.all_data) 
            #print self.all_data
            self.all_data = binascii.b2a_hex(self.all_data)
            #print ("thread_update_data=%s"%self.all_data)           
            event1.set()#更新完成，设event1为真
            
            #print("update serial finished")
    def stop(self):
        self.thread_stop = True
            

