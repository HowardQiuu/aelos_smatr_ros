import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#创建图像转换的类
class ImgConverter():
    def __init__(self):
        self.bridge = CvBridge()                                            #实例化 ROS 图像消息和 OpenCV 图像间转换的功能包
        #订阅名为 /usb_cam_chest/image_raw 的节点发布的消息, Image 为消息的队列大小，当消息传入时执行 cb_chest 回调函数
        self.sub_chest = rospy.Subscriber('/usb_cam_chest/image_raw', Image, self.cb_chest)
        self.img_chest = None                                               #清空消息队列
        

    def cb_chest(self, msg):                                                #将 ROS 的图像数据转换成 OpenCV 的图像格式程序
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_chest = cv2_img

    def chest_image(self):                                                  #得到采集图像信息的标志位和图像本身信息程序
        return True, self.img_chest
        