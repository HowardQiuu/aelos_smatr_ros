import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time

cv2_img = None

# ****************************************
#                 图片处理
# ****************************************

def get_img(camera):
    bridge = CvBridge()
    try:
        if camera == "head":
            msg = rospy.wait_for_message('/usb_cam_head/image_raw', Image)
        elif camera == "chest":
            msg = rospy.wait_for_message('/usb_cam_chest/image_raw', Image)
        else:
            raise Exception('设备指令错误')

        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        cv2_img = cv2.GaussianBlur(cv2_img, (3, 3), 0)
        return cv2_img
    except Exception as e:
        print(e)


def colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX):
    """获取最大轮廓
    """
    cv_image = get_img(camera)

    HSV = [(H_MIN, S_MIN, V_MIN ), (H_MAX , S_MAX , V_MAX)]

    aim_frame  = cv2.inRange(cv_image , *HSV)
    aim_frame  = cv2.erode(aim_frame , None, iterations=2)
    aim_frame  = cv2.dilate(aim_frame , np.ones((3, 3), np.uint8), iterations=2)

    # cv2.imwrite('./ball_mask.jpg', aim_frame )

    contours, hierachy2 = cv2.findContours(aim_frame , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=lambda c: cv2.contourArea(c), reverse=True)

    dts = cv2.drawContours(cv_image, contours, 0,(0, 0, 255),cv2.FILLED)
    # cv2.imwrite('./test.jpg', dts)

    return contours



# ****************************************
#                 对外开放接口
# ****************************************

def get_color_size(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX):
    """获得颜色面积
    """
    contours = colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)
    if len(contours) != 0:
        binary = cv2.contourArea(contours[0])
        return binary
    else:
        return 0

def get_color_percent(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX):
    """获得颜色面积占比
    """
    contours = colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)
    if len(contours) != 0:
        binary = cv2.contourArea(contours[0])            #轮廓的面积
    else:
        return 0
    cv_image = get_img(camera)
    if len(cv_image) != 0:
        im_area = cv_image.shape[0] * cv_image.shape[1]  #图片的面积
        return round((binary/im_area)*100,2)             #轮廓的面积占比
    else:
        return 0


def have_color(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX):
    """判断颜色是否存在
    """
    contours = colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)
    if len(contours) != 0:
        return True
    else:
        return False




def get_central_coordinate(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX):
    """获得单个颜色区域中心坐标
    """
    contours = colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)

    if len(contours) != 0:
        output = cv2.minAreaRect(contours[0])
        box = np.int0(cv2.boxPoints(output))
        # print (box, output)

        center_point = tuple(int(i) for i in map(lambda x, y: (x + y)/2, box[0], box[2]))
        return center_point
    else:
        return (-1,-1)


def get_frame(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX , return_value = "all"):
    contours = colour_Silhouettes(camera, H_MIN, S_MIN, V_MIN, H_MAX , S_MAX , V_MAX)
    if len(contours) != 0:
        rect = cv2.minAreaRect(contours[0])
        x, y, w, h = int(rect[0][0]), int(rect[0][1]), int(rect[1][0]), int(rect[1][1])
        if return_value == "x":
            return x
        elif return_value == "y":
            return y
        elif return_value == "w":
            return w
        elif return_value == "h":
            return h
        else:
            return x, y, w, h
    else:
        return False



if __name__ == "__main__":
    rospy.init_node("test")
    #colour_Silhouettes("head",10, 10, 10, 180, 190, 60)
    # print(get_color_size("head",10, 10, 10, 180, 190, 60))