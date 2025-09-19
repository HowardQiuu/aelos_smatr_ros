import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision


def main(CMDcontrol):
    bridge = CvBridge()

    # 网络模型
    base_options = core.BaseOptions(
            file_name='age_gender/gender_model.tflite', num_threads=1)
    
    # 模型 配置参数
    # 只输出最大的一个结果，结果要求相似度60%以上
    detection_options = processor.DetectionOptions(
            max_results=1, score_threshold=0.6)
    options = vision.ObjectDetectorOptions(
            base_options=base_options, detection_options=detection_options)
    
    
    # 加载人脸检测的网络和模型
    detector = vision.ObjectDetector.create_from_options(options)
    
    # 打开一个视频文件或一张图片或一个摄像头
    img_cv = rospy.wait_for_message('/usb_cam_head/image_raw', Image)
    img_cv = bridge.imgmsg_to_cv2(img_cv, "bgr8")
    # cv2.imwrite('./opencv.jpg', img_cv)

    # 按照TFLite模型的要求，将图像从BGR转换成RGB
    rgb_image = cv.cvtColor(img_cv, cv.COLOR_BGR2RGB)

    # 从RGB图像创建一个TensorImage对象
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # 运行人脸检测模型进行推理
    detection_result = detector.detect(input_tensor)
    det_result = detection_result.detections

    if det_result is None:
        print("No face Detected")

    for detection in det_result:
        # 获取人脸检测的结果
        category = detection.categories[0]
        gender = category.category_name
        print("gender = ",gender)
        if gender == "male":
            CMDcontrol.action_append("play_hello_sir")
            CMDcontrol.action_append("背手鞠躬")
        elif gender == "female":
            CMDcontrol.action_append("play_hello_Ladies")
            CMDcontrol.action_append("飞吻")

 

if __name__ == "__main__":
    rospy.init_node("opencv")

    pub = rospy.Publisher("face_image_raw", Image, queue_size=1)

    bridge = CvBridge()

    # 网络模型
    base_options = core.BaseOptions(
            file_name='age_gender/gender_model.tflite', num_threads=1)
    
    # 模型 配置参数
    # 只输出最大的一个结果，结果要求相似度60%以上
    detection_options = processor.DetectionOptions(
            max_results=1, score_threshold=0.6)
    options = vision.ObjectDetectorOptions(
            base_options=base_options, detection_options=detection_options)
    
    
    # 加载人脸检测的网络和模型
    detector = vision.ObjectDetector.create_from_options(options)
    
    while True:
        # 打开一个视频文件或一张图片或一个摄像头
        img_cv = rospy.wait_for_message('/usb_cam_head/image_raw', Image)
        img_cv = bridge.imgmsg_to_cv2(img_cv, "bgr8")
        # cv2.imwrite('./opencv.jpg', img_cv)

        # 按照TFLite模型的要求，将图像从BGR转换成RGB
        rgb_image = cv.cvtColor(img_cv, cv.COLOR_BGR2RGB)

        # 从RGB图像创建一个TensorImage对象
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # 运行人脸检测模型进行推理
        detection_result = detector.detect(input_tensor)
        det_result = detection_result.detections

        if det_result is None:
            print("No face Detected")

        for detection in det_result:
            # 获取人脸检测的结果
            category = detection.categories[0]
            gender = category.category_name
            x = detection.bounding_box.origin_x
            y = detection.bounding_box.origin_y
            w = detection.bounding_box.width
            h = detection.bounding_box.height

            score = round(category.score, 2)
            result_text = gender + ' ' + str(score)

            BGR_BLUE = (255, 127, 0)
            BGR_RED = (0, 57, 255)

            cv.rectangle(img_cv, (x, y), (x+w, y+h), BGR_BLUE, 2)
            cv.putText(img_cv, result_text, (10+x, 20+y), cv.FONT_ITALIC,
                        0.7, BGR_RED, 1)
        
        img = bridge.cv2_to_imgmsg(img_cv, "bgr8")
        pub.publish(img)