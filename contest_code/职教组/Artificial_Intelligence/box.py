import time
import cv2
import threading
import numpy as np
import rospy
import math

from image_converter import ImgConverter

#定义一些参数
Chest_img = None
ChestOrg = None

chest_box_x = None
chest_box_y = None
chest_box_angle = None
sign = 0
ID = 0
DEBUG = 0


# 不同色块的hsv范围
color_range = {
    'red': [(167 , 69 , 0 ), (180 , 255 , 255)],
    'orange': [( 18 , 57 , 94), ( 26 , 255 , 134 )]
}


#获取图像
def get_img():
    global Chest_img,ChestOrg
    global ret
    image_reader_chest = ImgConverter()
    while True:
        ret, ChestOrg = image_reader_chest.chest_image()
        time.sleep(1)
        if ChestOrg is not None:
            Chest_img = ChestOrg
            time.sleep(0.05)
        else:
            time.sleep(1)
            print("暂时未获取到图像")
th2 = threading.Thread(target=get_img)
th2.setDaemon(True)
th2.start()

# 查找方块
def find_box(img,color_name):
    global chest_box_x, chest_box_y ,chest_box_angle
    center_x = 0
    center_y = 0
    box_Y = 0
    box_X = 0

    if Chest_img is None:
        print('等待获取图像中...')
        time.sleep(1)
    else:
        center = []
        OrgFrame = img
        box_img = img
        box_img_bgr = cv2.cvtColor(box_img, cv2.COLOR_RGB2BGR)  # 将图片转换到BRG空间
        box_img_hsv = cv2.cvtColor(box_img, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        box_img = cv2.GaussianBlur(box_img_hsv, (3, 3), 0)  # 高斯模糊
        box_img_mask = cv2.inRange(box_img, color_range[color_name][0], color_range[color_name][1])  # 二值化
        box_img_closed = cv2.erode(box_img_mask, None, iterations=2)  # 腐蚀
        box_img_opened = cv2.dilate(box_img_mask, np.ones((4, 4), np.uint8), iterations=2)  # 膨胀    先腐蚀后运算等同于开运算
        (contours, hierarchy) = cv2.findContours(box_img_opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) != 0:
            area = []
            for cn in contours:
                contour_area = math.fabs(cv2.contourArea(cn))
                area.append(contour_area)
            max_index = np.argmax(area)
            
            rect = cv2.minAreaRect(contours[max_index])  # 最小外接矩形
            box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
            
            Ax = box[0, 0]
            Ay = box[0, 1]
            Bx = box[1, 0]
            By = box[1, 1]
            Cx = box[2, 0]
            Cy = box[2, 1]
            Dx = box[3, 0]
            Dy = box[3, 1]
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x = int((pt1_x + pt3_x) / 2)
            center_y = int((pt1_y + pt3_y) / 2)
            center.append([center_x, center_y])
            cv2.drawContours(OrgFrame, [box], -1, [0, 0, 255, 255], 3)
            cv2.circle(OrgFrame, (center_x, center_y), 10, (0, 0, 255), -1)  # 画出中心点
            # 求得大矩形的旋转角度，if条件是为了判断长的一条边的旋转角度，因为box存储的点的顺序不确定\
            if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                chest_box_angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0])) * 180.0 / math.pi
            else:
                chest_box_angle = - math.atan( (box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0]) ) * 180.0 / math.pi  # 负号是因为坐标原点的问题
            if center_y > box_Y:
                box_Y = center_y
            if center_x > box_X:
                box_X = center_x
            chest_box_y = box_Y
            chest_box_x = box_X

        if DEBUG:
            cv2.putText(OrgFrame, "chest_box_y:" + str(chest_box_y),
                        (10, OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        
            cv2.putText(OrgFrame, "chest_box_x:" + str(chest_box_x),
                        (10, OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        
            cv2.putText(OrgFrame, "chest_box_angle:" + str(chest_box_angle),
                        (10, OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                    
            cv2.imwrite('./OrgFrame.jpg', OrgFrame)

            print("chest_box_angle = ",chest_box_angle)
            print("chest_box_y = ",chest_box_y)
            print("chest_box_x = ",chest_box_x)



#去搬箱子
def goto_box(CMDcontrol):
    print('已经找到目标，准备调整位置')
    global ID
    time.sleep(1)
    if chest_box_x is None :
        print('等待中获取坐标中...')
    else:
        if chest_box_x < 295: 
            print("正在左侧移 ",chest_box_x)
            CMDcontrol.action_append("Left02move")
            time.sleep(0.5)
        elif chest_box_x > 365:    
            print("正在右侧移 ",chest_box_x)
            CMDcontrol.action_append("Right02move")
            time.sleep(0.5)
        else:
            if chest_box_y < 310:  
                print("前进",chest_box_y)
                CMDcontrol.action_append("Forwalk01")
                time.sleep(0.5)
            elif chest_box_y >= 360:  
                print("后退",chest_box_y)
                CMDcontrol.action_append("Back2Run")
                time.sleep(0.5)
            else:
                print("开始抱箱子")
                CMDcontrol.action_append("Forwalk01")
                CMDcontrol.action_append("box_up6")
                ID += 1
                #抱完箱子往前走几步
                time.sleep(1)
                CMDcontrol.action_append("boxForward")
                time.sleep(1)
                CMDcontrol.action_append("boxForward")


#移动到目标区
def goto_target_zone(CMDcontrol):
    print('已经找到目标区，准备调整位置')
    global sign
    time.sleep(2)
    if chest_box_angle is None :
        print('等待中获取坐标中...')
    else:
        if chest_box_y < 300:
            if chest_box_angle > 10:
                print("左转")
                CMDcontrol.action_append("boxturn009L")
            elif chest_box_angle < -10:
                print("右转")
                CMDcontrol.action_append("boxturn009R")
            elif chest_box_x < 260:  
                print("正在左侧移 ",chest_box_x)
                CMDcontrol.action_append("boxLeft")
            elif chest_box_x > 360:  
                print("正在右侧移 ",chest_box_x)
                CMDcontrol.action_append("boxRight")
            else:
                print("前进",chest_box_y)
                CMDcontrol.action_append("boxForward")
        else:
            print("开始放箱子")
            CMDcontrol.action_append("boxForward")
            CMDcontrol.action_append("boxDown")
            sign = 1

def main(CMDcontrol):
    global sign, ID
    step = 1
    while True:
        while ChestOrg is None:
            print('wite')
        if step == 1:  #转身寻找方块
            print('启动')
            CMDcontrol.action_append("fastForward03")
            while True:
                time.sleep(1)
                if ID == 0:
                    find_box(Chest_img, 'red')
                    goto_box(CMDcontrol)
                elif ID == 1:
                    find_box(Chest_img, 'orange')
                    goto_target_zone(CMDcontrol)
                    if sign == 1:
                        ID = 0
                        sign = 0
                        return

if __name__ == '__main__':
    rospy.init_node('image_listener')
    time.sleep(1)
    main()
