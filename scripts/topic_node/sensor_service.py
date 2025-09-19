#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布/sensor_info话题，自定义消息类型sensor_topic

import rospy
from aelos_smart_ros.msg import sensor_topic
import RPi.GPIO as GPIO
from aelos_smart_ros.srv import *
import sensor_magnet as sensor
import sensor_gpio as gpio_sensor
import threading
import time

import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import music

class topic_sensor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.mag_sensor = sensor.MagneticSensor()
        self.PINset = gpio_sensor.GPIOModule()
        self.person_msg = sensor_topic()   # 初始化sensor_topic类型的消息
        self.io = 0
        self.vol = 0
        self.mag_sign = True  
        self.adc = 0
        self.io_status = 'input'
        self.mag_status = 0x03
        self.mag_revise = True


    def publisher_sensor(self):
        # 创建一个Publisher，发布名为/sensor_info的topic，消息类型为sensor_topic，队列长度10
        person_info_pub = rospy.Publisher('/sensor_info', sensor_topic, queue_size=10)

        #设置循环的频率
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():   
            if self.PINset.gpio_status[16] == 0:
                self.person_msg.io2_status = "input"
                self.person_msg.IO_2 = self.PINset.read_io(2)

            if self.PINset.gpio_status[16] == 1:
                self.person_msg.io2_status = "output"
                self.person_msg.IO_2 = self.PINset.gpio_level[16]

            if self.PINset.gpio_status[20] == 0:
                self.person_msg.io1_status = "input"
                self.person_msg.IO_1 = self.PINset.read_io(1)

            if self.PINset.gpio_status[20] == 1:
                self.person_msg.io1_status = "output"
                self.person_msg.IO_1 = self.PINset.gpio_level[20]
            if self.mag_sign:
                try:
                    self.person_msg.mag = self.mag_sensor.get_bearing()
                    self.person_msg.mag_x = self.mag_sensor.mag_value_x
                    self.person_msg.mag_y = self.mag_sensor.mag_value_y
                    self.person_msg.mag_z = self.mag_sensor.mag_value_z
                except Exception as e:
                    print(e)
                    continue
                

            # 发布消息
            person_info_pub.publish(self.person_msg)
            # rospy.loginfo("Publsh person message[%s, %d, %d]", 
            #         person_msg.name, person_msg.age, person_msg.sex)

            # 按照循环频率延时
            rate.sleep()



    def io_service(self,req):
        if "%s"%req.status == "input":
            #设置io端口为输入模式
            self.PINset.change_to_input(self.PINset.port_map.get(int("%s"%req.io)))
        if "%s"%req.status == "output":
            #设置io端口为输出模式
            self.io = int("%s"%req.io)
            self.vol = int("%s"%req.vol)
            self.PINset.set_vol(self.io,self.vol)
        return [True]


    def magnet_revise(self):
        self.mag_sensor.mag_check(True)
        time.sleep(2)
        self.mag_revise = True

    def magnet_calibration(self):
        self.mag_sensor.mag_check_calibrate(True)

    def magnet_service(self,req):
        #校正地磁数据
        if "%s"%req.mag == 'revise':
            if self.mag_revise:
                self.mag_status = 0x01
                threading.Thread(target=self.magnet_revise).start()
                self.mag_revise = False
                return self.mag_status
            self.mag_status = self.mag_sensor.get_mag_status()
        if "%s"%req.mag == 'start_calibrate':
            threading.Thread(target=self.magnet_calibration).start()
            self.mag_status = self.mag_sensor.get_mag_status()
        if "%s"%req.mag == 'stop_calibrate':
            self.mag_status = self.mag_sensor.get_mag_status()
            if self.mag_status == 0x02:
                self.mag_sensor.wait_stop_mag_check()
                self.mag_status= self.mag_sensor.get_mag_status()

        if "%s"%req.mag == 'get_magnet_status':
            self.mag_status= self.mag_sensor.get_mag_status()
            
        return self.mag_status

    
    def read_magnet(self,req):
        if "%s"%req.data == 'get':
            mag = self.mag_sensor.get_mag_values()
        return mag



    def read_io_status(self,req):
        gpio = self.PINset.get_gpio_status(int("%s"%req.io))
        self.io_status = gpio
        if gpio == 0:   #input
            self.adc = self.PINset.read_io(int("%s"%req.io))
        else:   #output
            self.adc = self.PINset.get_gpio_level(int("%s"%req.io))
        return self.io_status,self.adc


    def init_service(self):
        rospy.Service('io_service', topic, self.io_service)
        rospy.Service('magnet_service', revise_mag, self.magnet_service)
        rospy.Service('get_magnet', get_mag, self.read_magnet)
        rospy.Service('get_io', get_io, self.read_io_status)



if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('publisher', anonymous=True)
    publisher = topic_sensor()
    publisher.init_service()
    publisher.publisher_sensor()