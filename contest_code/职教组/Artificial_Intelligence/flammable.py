#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# from color_filter import color_filter

head_ret = True     # 读取图像标志位
head_img = None   # 原始图像更新


ros_view_debug = True
sign = 0
Debug = 0


class ImgConverter():
    def __init__(self):
        self.bridge = CvBridge()        
        self.sub_head = rospy.Subscriber('/usb_cam_head/image_raw', Image, self.cb_head)
        self.img_head = None
        

    def cb_head(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_head = cv2_img

    def head_image(self):
        return True, self.img_head


################################################读取图像线程#################################################

def get_img():
    global head_img, head_ret

    image_reader_head = ImgConverter()
    while not rospy.is_shutdown():
        head_ret, head_img = image_reader_head.head_image()
        time.sleep(0.05)


# 读取图像线程

th1 = threading.Thread(target=get_img)
th1.setDaemon(True)
th1.start()


#################################################################HSV颜色####################################

color_dist = {
              'cylinder_red': {'Lower': np.array([0 , 62 , 0]), 'Upper': np.array([20 , 255 , 255])}
              }


################################################拆除易燃物########################################

head_cylinder_angle = 90

head_cylinder_flag = False


def Perform(CMDcontrol):
    global head_cylinder_angle
    global head_cylinder_x, head_cylinder_y 
    global head_cylinder_flag
    global sign

    if head_cylinder_flag == True:
        time.sleep(3)
        if width < 130:
            if width < 80:
                if head_cylinder_x <= 230:
                    print("head_cylinder_x < 230 左侧移 ",head_cylinder_x)
                    CMDcontrol.action_append("Left02move")
                elif head_cylinder_x >= 280:
                    print("head_cylinder_x > 280 右侧移 ",head_cylinder_x)
                    CMDcontrol.action_append("Right02move")
                else:
                    print("width < 80 快走前进 ",width)
                    CMDcontrol.action_append("FastForward1s")
            else:
                print("width < 130 慢走前进 ",width)
                CMDcontrol.action_append("Forwalk02")
        else:
            print("拆除",width,head_cylinder_x)
            CMDcontrol.action_append("Forwalk01")
            CMDcontrol.action_append("拆除右侧易燃物")
            CMDcontrol.action_append("play_移除易燃物")
            sign = 1

    else:
        print("未发现拆除物，前进")
        CMDcontrol.action_append("Forwalk02")



          ###################################### 图像处理 #####################################

def dismantler(CMDcontrol):
    global head_cylinder_angle
    global head_cylinder_x, head_cylinder_y ,width
    global head_cylinder_flag
    global head_img, sign

    # 初始化
    sum_contours = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    #开始先快走3步
    CMDcontrol.action_append("fastForward03")
    time.sleep(0.5)

    while not rospy.is_shutdown():
        headOrg = head_img.copy()
        
        cylinder_OrgFrame = headOrg.copy()

        box_img = cylinder_OrgFrame
        box_img_bgr = cv2.cvtColor(box_img, cv2.COLOR_RGB2BGR)  # 将图片转换到BRG空间
        box_img_hsv = cv2.cvtColor(box_img, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        box_img = cv2.GaussianBlur(box_img_hsv, (3, 3), 0)  # 高斯模糊
        box_img_mask = cv2.inRange(box_img, color_dist['cylinder_red']['Lower'], color_dist['cylinder_red']['Upper'])  # 二值化
        box_img_closed = cv2.erode(box_img_mask, None, iterations=2)  # 腐蚀
        box_img_opened = cv2.dilate(box_img_mask, np.ones((4, 4), np.uint8), iterations=2)  # 膨胀    先腐蚀后运算等同于开运算
        (contours, hierarchy) = cv2.findContours(box_img_opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) != 0:
            head_cylinder_flag = True
            area = []
            for cn in contours:
                contour_area = math.fabs(cv2.contourArea(cn))
                area.append(contour_area)
            max_index = np.argmax(area)
            x, y, width, height = cv2.boundingRect(contours[max_index])
            head_cylinder_x = x + width/2
            # print("head_cylinder_x = ",head_cylinder_x , "width = ",width)
            cv2.rectangle(cylinder_OrgFrame, (x, y), (x + width, y + height), (0, 255, 0), 2)

            if Debug:
                cv2.imwrite('image.png',cylinder_OrgFrame)
                # cv2.imshow("image", cylinder_OrgFrame)
                # cv2.waitKey(2000)
            Perform(CMDcontrol)
        else:
            head_cylinder_flag = False
            width = 0
        if sign == 1:
            sign = 0
            break



# 踢球进洞
if __name__ == '__main__':
    rospy.init_node('flammable')
    time.sleep(2)
    dismantler()
