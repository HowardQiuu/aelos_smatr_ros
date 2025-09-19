#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy
import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")        
from leju import *

from image_converter import ImgConverter


img_debug = 0
action_DEBUG = False

chest_ret = False     # 读取图像标志位
ret = False           # 读取图像标志位
ChestOrg_img = None   # 原始图像更新
HeadOrg_img = None    # 原始图像更新
ChestOrg_copy = None
HeadOrg_copy = None

sleep_time_s = 0.01
sleep_time_l = 0.05
real_test = 1
reset = 0


rgb_hole = [0,113,150]
rgb_hole_thred = 18
rgb_ball = [88,34,25]
rgb_ball_thred = 16

ros_view_debug = True


################################################读取图像线程#################################################

def get_img():
    global ChestOrg_img, chest_ret

    image_reader_chest = ImgConverter()
    while True:
        chest_ret, ChestOrg_img = image_reader_chest.chest_image()
        time.sleep(0.05)



color_dist = { 
              'ball_red': {'Lower': np.array([0 , 169 , 0]), 'Upper': np.array([17 , 255 , 255])},
              'blue_hole': {'Lower': np.array([102 , 186 , 168]), 'Upper': np.array([115 , 251 , 255])},
              }


def getAreaMaxContour2(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  
                area_max_contour = c
    return area_max_contour




################################################第七关：踢球########################################

golf_angle_ball = 90
Chest_ball_angle = 90
hole_Angle = 45
golf_angle = 0
ball_x = 0
ball_y = 0
golf_angle_flag = False
golf_dis_start = True
golf_angle_start = False
golf_ok = False
hole_flag = False
Chest_ball_flag = False
Chest_golf_angle = 0

ball_dis_start = True
hole_angle_start = False

head_state = 0

hole_x = 0
hole_y = 0

angle_dis_count = 0
picnum = 0
fast_run = True

def act_move():
    global step, state, reset, skip
    global hole_Angle,ball_hole
    global golf_angle_ball, golf_angle ,Chest_ball_angle ,Chest_golf_angle
    global ball_x, ball_y,Chest_ball_x, Chest_ball_y 
    global golf_angle_flag
    global golf_angle_start, golf_dis_start
    global golf_ok
    global hole_flag,Chest_ball_flag
    global ball_dis_start,hole_angle_start
    global head_state, angle_dis_count ,fast_run


    if True:
        # print("step==",step)
        # print("--Chest_ball_x",Chest_ball_x,"Chest_ball_y",Chest_ball_y," hole_Angle",hole_Angle," Chest_ball_angle",Chest_ball_angle)
        #此版本的横轴是x 纵轴是y 球越往右x越大 球离机器人越近y越大
        if step == 0:
            if Chest_ball_flag == True:
                if fast_run:
                    if Chest_ball_y <= 320:
                        print("前进 Forwalk02 ",Chest_ball_y)
                        base_action.action("Forwalk02")
                        # X
                        if Chest_ball_x > 290:
                            print("161L Chest_ball_x ", Chest_ball_x, " > 290 右侧移 ")
                            base_action.action("Right3move")
                        elif Chest_ball_x < 200:
                            print("159L Chest_ball_x ", Chest_ball_x, " < 200 左侧移 ")
                            base_action.action("Left3move")
                    else:
                        print("快走完成",Chest_ball_y)
                        fast_run = False

                else:
                    if Chest_ball_y < 350:
                        # X
                        if Chest_ball_x > 300:
                            print("161L Chest_ball_x ", Chest_ball_x,"  > 300 右侧移 ")
                            base_action.action("Right3move")
                        elif Chest_ball_x < 210:
                            print("159L Chest_ball_x ", Chest_ball_x,"  < 210 左侧移 ")
                            base_action.action("Left3move")
                        else:
                            print("前挪一点点",Chest_ball_y)
                            base_action.action("Forwalk01")
                    else:
                        print("goto step1  ",Chest_ball_y)
                        step = 1
            else:
                print("未发现红球,前进,Forwalk02")
                base_action.action("Forwalk02")
                
        elif step == 1:
            if Chest_ball_y <= 360:
                print("前挪一点点 Chest_ball_y ", Chest_ball_y, " < 360 ")
                base_action.action("Forwalk00")
            elif Chest_ball_y > 390:
                print("1903L 后一步 Chest_ball_y ", Chest_ball_y ," > 390")
                base_action.action("Back2Run")
            elif 360< Chest_ball_y <= 390:
                step = 2


        elif step == 2:
            if ball_dis_start:
                if Chest_ball_x <= 210:
                    if Chest_ball_x < 190:
                        print("373L4 需要左侧移 Left3move", Chest_ball_x)
                        base_action.action("Left3move")
                    else:
                        print("376L4 需要左侧移 Left02move", Chest_ball_x)
                        base_action.action("Left02move")
                    angle_dis_count = 0
                elif Chest_ball_x > 320:
                    if Chest_ball_x > 340:
                        print("359L4 需要右侧移 Right3move", Chest_ball_x)
                        base_action.action("Right3move")
                    else:
                        print("384L4 需要右侧移 Right02move", Chest_ball_x)
                        base_action.action("Right02move")
                    angle_dis_count = 0
                else:
                    print("388L4 Chest_ball_x---位置ok")
                    ball_dis_start = False
                    hole_angle_start = True
            if hole_angle_start:
                if hole_Angle <=0:    
                    # angle
                    if hole_Angle < 0:
                        #if hole_Angle <= -10:
                        if Chest_ball_y > 388:
                            print("392L4 需要后挪一点 Back2Run ",Chest_ball_y)
                            base_action.action("Back2Run")
                            angle_dis_count = 0
                        elif Chest_ball_y < 340:
                            print("395L4 需要前挪一点 Forwalk00",Chest_ball_y)
                            base_action.action("Forwalk00")
                            angle_dis_count = 0

                        print("381L4 大左转一下  turn004L ", hole_Angle)
                        base_action.action("turn004L")
                        angle_dis_count = 0
                    else:
                        print("401L4 hole_Angle---角度ok")
                        angle_dis_count = angle_dis_count + 1
                        ball_dis_start = True
                        hole_angle_start = False

                if hole_Angle > 0:       
                    # angle
                    if hole_Angle > 10:
                        if hole_Angle > 40:
                            if Chest_ball_y > 380:
                                print("409L4 需要后挪一点 Back2Run ",Chest_ball_y)
                                base_action.action("Back2Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 340:
                                print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                                base_action.action("Forwalk00")
                                angle_dis_count = 0

                            print("250L4 大右转一下 turn004R ", hole_Angle)
                            base_action.action("turn004R")
                        else:
                            if Chest_ball_y > 380:
                                print("421L4 需要后挪一点 Back1Run ",Chest_ball_y)
                                base_action.action("Back1Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 340:
                                print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                                base_action.action("Forwalk00")
                                angle_dis_count = 0

                            print("352L4 右转一下 turn001R ", hole_Angle)
                            base_action.action("turn001R")
                    elif hole_Angle < 0:
                        #if hole_Angle < 
                        if Chest_ball_y > 380:
                                print("421L4 需要后挪一点 Back1Run ",Chest_ball_y)
                                base_action.action("Back1Run")
                                angle_dis_count = 0
                        elif Chest_ball_y < 340:
                            print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                            base_action.action("Forwalk00")
                            angle_dis_count = 0

                        print("352L4 左转一下 turn001L ", hole_Angle)
                        base_action.action("turn001L")
                    else:
                        print("417L4 hole_Angle---角度OK")         
                        angle_dis_count = angle_dis_count + 1
                        ball_dis_start = True
                        hole_angle_start = False

                if angle_dis_count > 3:
                    angle_dis_count = 0
                    print("step step 5555")
                    step = 3
                    

        elif step == 3:
            if ball_dis_start:  # 390<y<450  230<x<250
                if Chest_ball_x < 220:
                    print("446L 需要左侧移 Left1move", Chest_ball_x)
                    base_action.action("Left1move")
                    angle_dis_count = 0
                elif Chest_ball_x > 300:
                    print("454L 需要右侧移 Right1move", Chest_ball_x)
                    base_action.action("Right1move")
                    angle_dis_count = 0
                else:
                    print("340L Chest_ball_x---位置ok")
                    ball_dis_start = False
                    hole_angle_start = True
            if hole_angle_start:
                if hole_Angle <0:
                    # angle
                    if hole_Angle < 0:
                        # y
                        if Chest_ball_y > 380:
                            print("475L 需要后挪一点 Back1Run ",Chest_ball_y)
                            base_action.action("Back1Run")
                            angle_dis_count = 0
                        elif Chest_ball_y < 340:
                            print("368L 需要前挪一点 Forwalk00",Chest_ball_y)
                            base_action.action("Forwalk00")
                            angle_dis_count = 0

                        if hole_Angle <= -10:
                            print("465L 大左转一下  turn001L ", hole_Angle)
                            base_action.action("turn001L")
                        else:
                            print("468L 左转一下  turn001L ", hole_Angle)
                            base_action.action("turn001L")
                    else:
                        print("471L hole_Angle---角度ok")
                        angle_dis_count = angle_dis_count + 1

                    ball_dis_start = True
                    hole_angle_start = False
                if hole_Angle >0:
                    # angle
                    if hole_Angle > 15:
                        
                        if Chest_ball_y > 380:
                            print("475L 需要后挪一点 Back1Run ",Chest_ball_y)
                            base_action.action("Back1Run")
                            angle_dis_count = 0
                        elif Chest_ball_y < 340:
                            print("368L 需要前挪一点 Forwalk00 ",Chest_ball_y)
                            base_action.action("Forwalk00")
                            angle_dis_count = 0

                        if hole_Angle > 40:
                            print("479L 大右转一下 turn001R ", hole_Angle)
                            base_action.action("turn001R")
                        else:
                            print("482L 右转一下 turn001R ", hole_Angle)
                            base_action.action("turn001R")
                    elif hole_Angle < -5:
                        if Chest_ball_y > 380:
                                print("421L4 需要后挪一点 Back1Run ",Chest_ball_y)
                                base_action.action("Back1Run")
                                angle_dis_count = 0
                        elif Chest_ball_y < 340:
                            print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                            base_action.action("Forwalk00")
                            angle_dis_count = 0

                        print("352L4 左转一下 turn001L ", hole_Angle)
                        base_action.action("turn001L")
                    
                    else:
                        print("485L hole_Angle---角度OK")               
                        angle_dis_count = angle_dis_count + 1

                    ball_dis_start = True
                    hole_angle_start = False

                if angle_dis_count > 2:
                    angle_dis_count = 0
                    step = 4


        elif step == 4:
            if Chest_ball_y <= 380:
                print("289L 向前挪动一点点 Forwalk00 ",Chest_ball_x)
                base_action.action("Forwalk00")
                if hole_Angle < 5:
                    print("468L 左转一下  turn001L ", hole_Angle)
                    base_action.action("turn001L")
                elif hole_Angle > 15:
                    print("482L 右转一下 turn001R ", hole_Angle)
                    base_action.action("turn001R")
            else:
                print("next step")
                step = 5

        elif step == 5:
            if Chest_ball_x < 220:
                print("412L 向左移动 Left1move")
                base_action.action("Left1move")
            elif Chest_ball_x > 300:
                print("410L 向右移动 Right1move")
                base_action.action("Right1move")
            elif Chest_ball_y < 400:
                print("289L 向前挪动一点点 Forwalk00 ",Chest_ball_y)
                base_action.action("Forwalk00")
                if hole_Angle < 5:
                    print("468L 左转一下  turn001L ", hole_Angle)
                    base_action.action("turn001L")
                elif hole_Angle > 15:
                    print("482L 右转一下 turn001R ", hole_Angle)
                    base_action.action("turn001R")
            elif Chest_ball_y > 420:
                print("2244L 向后挪动一点点 Back0Run ",Chest_ball_y)
                base_action.action("Back0Run")
                if hole_Angle < 5:
                    print("468L 左转一下  turn001L ", hole_Angle)
                    base_action.action("turn001L")
                elif hole_Angle > 15:
                    print("482L 右转一下 turn001R ", hole_Angle)
                    base_action.action("turn001R")
            else:
                print("414L 踢球踢球 LfootShot " ,Chest_ball_y)
                time.sleep(1)
                base_action.action("LfootShot")
                step = 6
                print("完成")
                base_action.action("Stand")
                base_action.action("Wanyao20")
                base_action.action("Stand")


def kick_ball():
    global state, state_sel, step, reset, skip
    global hole_Angle
    global golf_angle_ball, golf_angle ,Chest_ball_angle, Chest_golf_angle
    global ball_x, ball_y ,Chest_ball_x, Chest_ball_y 
    global hole_flag ,Chest_ball_flag
    global ChestOrg_img
    global picnum,img_debug

    # 初始化
    sum_contours = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    step = 0

    # 读取图像线程
    get_image_thread = threading.Thread(target=get_img)
    get_image_thread.setDaemon(True)
    get_image_thread.start()

    while chest_ret is False:
        time.sleep(1)
        continue

    while not rospy.is_shutdown():
        if 0<=step < 6:
            time.sleep(0.5)
            
            #cv.copy run error if img is None
            if ChestOrg_img is None:
                print("None img")
                continue
                
            ChestOrg = ChestOrg_img.copy()
            #ChestOrg=np.rot90(ChestOrg)
            
            Hole_OrgFrame = ChestOrg.copy()
            Ball_OrgFrame = ChestOrg.copy()

            img_h, img_w = Hole_OrgFrame.shape[:2]

            # 把上中心点和下中心点200改为640/2  fftest
            bottom_center = (int(240), int(img_h))  #图像底中点
            top_center = (int(240), int(0))     #图像顶中点

            # 开始处理图像
            Hole_hsv = cv2.cvtColor(Hole_OrgFrame, cv2.COLOR_BGR2HSV)
            Hole_hsv= cv2.GaussianBlur(Hole_hsv, (3, 3), 0)

            Hole_Imask = cv2.inRange(Hole_hsv, color_dist['blue_hole']['Lower'], color_dist['blue_hole']['Upper'])
            Hole_Imask = cv2.erode(Hole_Imask, None, iterations=8)
            Hole_Imask = cv2.dilate(Hole_Imask, np.ones((3, 3), np.uint8), iterations=8)

            
            # cv2.imshow('hole_mask', Hole_Imask)      # hole mask
            # print('Press a key to continue:')
            # cv2.waitKey(0)


            # 初始化
            hole_center = [0, 0]
            Chest_ball_center = [0, 0]


          # chest 球洞处理
            hole_x = 0
            hole_y = 0

            cnts, hierachy = cv2.findContours(Hole_Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    #**获得图片轮廓值  #遍历图像层级关系
            # cnts = color_filter(Hole_OrgFrame, rgb_hole, thred=rgb_hole_thred)

            # print(cnts)
            # *取得一个球洞的轮廓*
            for i in range(0, len(cnts)):               # 初始化sum_contours，使其等于其中一个c，便于之后拼接的格式统一
                area = cv2.contourArea(cnts[i])         #计算轮廓面积
                # print("area : ",area)
                if img_debug:
                    cv2.putText(Hole_OrgFrame, "area:" + str(area),(10, Hole_OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                    cv2.waitKey(1)
                if 640 * 480 * 0.0005 < area < 640 * 480 * 0.45:  # 去掉很小的干扰轮廓以及最大的图像边界
                    # cv2.drawContours(Hole_OrgFrame, cnts, -1, (0, 255, 0), 3)
                    sum_contours = cnts[i]
                    break
                else:
                    # cv2.drawContours(Hole_OrgFrame, cnts, -1, (0, 0, 255), 3)
                    continue
            for c in cnts:
                area = cv2.contourArea(c)                                       #计算轮廓面积
                if 640 * 480 * 0.0005 < area < 640 * 480 * 0.45:
                    sum_contours = np.concatenate((sum_contours, c), axis=0)    #数组拼接
                    # cv2.drawContours(Hole_OrgFrame, c, -1, (0, 255, 0), 3)
                else:
                    # cv2.drawContours(Hole_OrgFrame, c, -1, (0, 0, 255), 3)
                    continue
            sum_area = cv2.contourArea(sum_contours)    #计算轮廓面积
            if sum_area > 3:
                cnt_large = sum_contours
            else:
                cnt_large = None

            if cnt_large is not None:
                hole_flag = True
                (hole_x, hole_y), radius = cv2.minEnclosingCircle(cnt_large)    #最小内接圆形
                hole_center = (int(hole_x), int(hole_y))
                radius = int(radius)
                if (hole_center[0] - bottom_center[0]) == 0:
                    hole_Angle = 90
                else:
                    # hole_Angle  (y1-y0)/(x1-x0)
                    #hole_Angle = - math.atan((hole_center[1] - bottom_center[1]) / (hole_center[0] - bottom_center[0])) * 180.0 / math.pi
                    hole_Angle = - math.atan((hole_center[0] - bottom_center[0]) / (hole_center[1] - bottom_center[1])) * 180.0 / math.pi
            else:
                hole_flag = False

            if img_debug:
                cv2.putText(Hole_OrgFrame, "step:" + str(step),
                            (10, Hole_OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_angle:" + str(hole_Angle),
                            (10, Hole_OrgFrame.shape[0] - 115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_x:" + str(hole_x),
                            (10, Hole_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_y:" + str(hole_y),
                            (220, Hole_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_flag:" + str(hole_flag),
                            (10, Hole_OrgFrame.shape[0] - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.imshow("Hole_OrgFrame", Hole_OrgFrame)
                cv2.waitKey(10)


          # chest 红球处理
            Chest_ball_x = 0
            Chest_ball_y = 0


            Chest_Ball_hsv = cv2.cvtColor(Ball_OrgFrame, cv2.COLOR_BGR2HSV)
            Chest_Ball_hsv = cv2.GaussianBlur(Chest_Ball_hsv, (3, 3), 0)
            
            Chest_Ball_Imask = cv2.inRange(Chest_Ball_hsv, color_dist['ball_red']['Lower'], color_dist['ball_red']['Upper'])
            Chest_Ball_Imask = cv2.erode(Chest_Ball_Imask, None, iterations=2)
            Chest_Ball_Imask = cv2.dilate(Chest_Ball_Imask, np.ones((3, 3), np.uint8), iterations=2)
            
            # cv2.imshow('ball_mask', Chest_Ball_Imask)    # ball mask
            # print('Press a key to continue:')
            # cv2.waitKey(0)

            cnts2, hierachy2 = cv2.findContours(Chest_Ball_Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cnts2 = color_filter(Ball_OrgFrame, rgb_ball, thred=rgb_ball_thred)

            if cnts2 is not None:
                cnt_large3 = getAreaMaxContour2(cnts2, 10)
            else:
                print("1135L cnt_large is None")
                continue
            
            # 圆球轮廓  计算角度 Chest_ball_angle
            if cnt_large3 is not None:
                Chest_ball_flag = True
                (Chest_circle_x, Chest_circle_y), Chest_radius = cv2.minEnclosingCircle(cnt_large3)
                Chest_ball_center = (int(Chest_circle_x), int(Chest_circle_y))
                Chest_radius = int(Chest_radius)
                # cv2.circle(Ball_OrgFrame, Chest_ball_center, Chest_radius, (100, 200, 20), 2)
                # cv2.line(Ball_OrgFrame, Chest_ball_center, top_center, (0, 100, 0), 2)
                # ellipse = cv2.fitEllipse(cnt_large)
                # cv2.ellipse(OrgFrame,ellipse,(255,255,0),2)
                if (Chest_ball_center[0] - top_center[0]) == 0:
                    Chest_ball_angle = 90
                else:
                    # *Chest_ball_angle*  (y1-y0)/(x1-x0)
                    #Chest_ball_angle = - math.atan((Chest_ball_center[1] - top_center[1]) / (Chest_ball_center[0] - top_center[0])) * 180.0 / math.pi
                    Chest_ball_angle = - math.atan((Chest_ball_center[0] - top_center[0]) / (Chest_ball_center[1] - top_center[1])) * 180.0 / math.pi
                Chest_ball_x = int(Chest_circle_x)   # *ball_x*
                Chest_ball_y = int(Chest_circle_y) # *ball_y*
            else:
                Chest_ball_flag = False
                Chest_ball_y = 0

        if ros_view_debug:
            image_debug = cv2.circle(Ball_OrgFrame, (int(hole_x), int(hole_y)), 10, (255,0,0), 2)
            image_debug = cv2.circle(image_debug, (Chest_ball_x, Chest_ball_y), 10, (0,0,255), 2)
            cv2.imwrite('./debug.jpg', Ball_OrgFrame)
            cv2.imwrite('./raw_debug.jpg', ChestOrg)
        act_move()
        if step == 6:
            return 




# 踢球进洞
if __name__ == '__main__':
    
    # 读取图像线程
    th1 = threading.Thread(target=get_img)
    th1.setDaemon(True)
    th1.start()

    rospy.init_node('kickballnodes')
    time.sleep(3)
    base_action.action("Stand")
    kick_ball()