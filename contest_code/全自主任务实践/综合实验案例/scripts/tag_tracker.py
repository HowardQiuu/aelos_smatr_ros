import rospy                                                                #导入 python 下 ros 函数
import tf                                                                   #导入坐标系变换功能包
import time                                                                 #导入时间功能包
import threading                                                            #导入线程功能包
import math                                                                 #导入数学库
from ar_track_alvar_msgs.msg import AlvarMarkers                            #导入 ar_track_alvar 功能包的消息文件
import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")        
from leju import *
from flammable import dismantler                                             #导入拆除易燃物模块  

#创建获取Tag信息的类
class TagConverter():
    def __init__(self):
        #订阅名为 /chest/ar_pose_marker 的节点发布的消息, AlvarMarkers 为消息的队列大小，当消息传入时执行 sub_cb 回调函数
        self.sub = rospy.Subscriber('/chest/ar_pose_marker', AlvarMarkers, self.sub_cb)
        self.markers = []

    def sub_cb(self, msg):
        self.markers = []
        for marker in msg.markers:                                          #循环获取传入消息的 markers 类并赋值给 marker
            pos = marker.pose.pose.position                                 #获取相对 tag 的位置信息
            quat = marker.pose.pose.orientation                             #获取相对 tag 的四元数

            rpy = tf.transformations.\
                    euler_from_quaternion([quat.x, quat.y, quat.z, quat.w]) #将四元数转换成欧拉角
            rpy_arc = [0, 0, 0]
            for i in range(len(rpy)):
                rpy_arc[i] = rpy[i] / math.pi * 180                         #将欧拉角的弧度制转换成度
            
            # print("poseX--poseY--rpy_y:", pos.x, ",", pos.y, ",",rpy_arc[2]+90)       #用于测试标点

            self.markers.append([marker.id, pos.x, pos.y, rpy_arc[2]])      #将 Tag_id 、 x 方向距离、 y 方向距离，旋转角以组的形式添加到 markers 列表末尾

    def get_markers(self):                                                  #获取 Tag 信息程序
        return self.markers

    def get_nearest_marker(self):                                           #获取 Tag_id 最小值的那一组数据程序
        min_id = 999
        min_idx = 0
        markers = []
        for i in range(50):                                                 #将 markers 列表内添加50组 Tag 数据
            time.sleep(0.01)
            markers += self.markers
        
        for index, m in enumerate(markers):                                 #index 为每一组的索引，通过循环遍历的的方式找到 Tag_id 最小的那一组索引
            if m[0] < min_id:
                min_idx = index
                min_id = m[0]
        if min_id == 999:
            return []
        else:
            return markers[min_idx]


"""
机器人定位程序,根据传入图像获取到的 Tag 坐标信息,进行位置矫正,使机器人到达指定位置
@param{number} dis_x 是实时获取的 Tag_x 方向信息    
@param{number} dis_y 是实时获取的 Tag_y 方向信息 
@param{number} theta 是实时获取的 Tag 角度信息
@param{number} x_aim 是目标位置的 Tag_x 方向信息 
@param{number} y_aim 是目标位置的 Tag_y 方向信息   
@param{number} theta_aim 是目标位置的 Tag 角度信息
@param{number} x_threshold 是 Tag_x 方向允许的误差值
@param{number} y_threshold 是 Tag_y 方向允许的误差值
@param{number} theta_threshold 是 Tag 角度允许的误差值
@return{number} is_turn_done 若为 True 表示基于 Tag 的位置矫正完成, is_turn_done 若为 False 表示基于 Tag 的位置矫正未完成
"""
def turn_to_tag(dis_x, dis_y, theta, x_aim=0, y_aim=0, theta_aim=0, x_threshold=0.03, y_threshold=0.02, theta_threshold=5):
    is_turn_done = False                                                    #默认未对准 Tag
    
    x_offset = dis_x-x_aim                                                  #x 方向的偏离量
    y_offset = dis_y-y_aim                                                  #y 方向的偏离量
    theta_offset = theta-theta_aim                                          #角度的偏离量


    if (x_offset<x_threshold-0.03):                                          
        print("后退")
        base_action.action("Back2Run")

    elif (theta_offset-theta_threshold < -35):
        print("1右转")
        base_action.action("turn004R")
    elif (theta_offset+theta_threshold > 35):
        print("1左转")
        base_action.action("turn004L")

    elif (x_offset > x_threshold+0.2):
        print("前进")
        base_action.action("fastForward03")

    elif (x_offset > x_threshold+0.09):
        print("前进")
        base_action.action("Forwalk02")

    elif (y_offset < -y_threshold-0.05):
        print("1右移动  ")
        base_action.action("Right3move")
    elif (y_offset > y_threshold+0.05):
        print("1左移动 ")
        base_action.action("Left3move")

    elif (theta_offset < -theta_threshold):
        print("1右转")
        base_action.action("turn001R")
    elif (theta_offset > theta_threshold):
        print("1左转")
        base_action.action("turn001L")
    
    elif (y_offset < -y_threshold):
        print("2右移动  ")
        base_action.action("Right02move")
    elif (y_offset > y_threshold):
        print("2左移动 ")
        base_action.action("Left02move")

    elif (x_offset > x_threshold+0.06):
        print("前进")
        base_action.action("Forwalk01")
    

    else:
        print("turn to tag ok")
        print("dis_x**:",dis_x,"dis_y**:",dis_y,"theta**:",theta)
        is_turn_done = True

    return is_turn_done



def main():
    try:
        rospy.init_node('ar_tag_tracker')                                   #创建一个名为 ar_tag_tracker 的节点
        tag = TagConverter()                                                #实例化 Tag 类
        base_action.action('fastForward03')

        stage = 'start_door'                                                #默认处于起始关卡

        print('Start action thread ...')                                    
        time.sleep(1)


        while not rospy.is_shutdown():                                      #如果节点没有关闭将一直循环
            time.sleep(0.1)

            marker = tag.get_nearest_marker()                               #获取 Tag_id 最小值的那一组数据
            if len(marker) == 0:                                            #针对没采集到 Tag 信息进行处理
                if stage == 'start_door':                                   #若没有采集到 Tag 信息并且处于起始点，执行前进
                    base_action.action('Forwalk00')

                if stage == 'stage_one':                                    #若没有采集到 Tag 信息并且处于避障关卡的第一部分，执行向右移动
                    base_action.action('Right02move')

                if stage == 'stage_two':                                    #若没有采集到 Tag 信息并且处于避障关卡的第二部分，执行向左移动
                    base_action.action('Left02move')

                if stage == 'stage_bridge':                                 #若没有采集到 Tag 信息并且处于独木桥的关卡，执行后退
                    base_action.action('Back2Run')

                continue

            robot_tag_x = marker[1]                                         #Tag 的 x 信息在列表的第 2 个
            robot_tag_y = marker[2]                                         #Tag 的 y 信息在列表的第 3 个
            tag_yaw = marker[3] + 90                                        #Tag 的旋转角度信息在列表的第 4 个

            if marker[0] == 0:                                              #处于起始关卡
                stage = 'stage_one'
                result = turn_to_tag(robot_tag_x,robot_tag_y,tag_yaw,0.05,0.12,0)    #对 Tag_id:0，若返回值为 True，则执行下一关
                if (result == False):
                    continue
                
                base_action.action('Right3move')
                base_action.action('Right3move')
                base_action.action('Right3move')
                base_action.action('Right3move')
                base_action.action('Right3move')
                
            elif marker[0] == 1:
                stage = 'stage_two'
                result = turn_to_tag(robot_tag_x,robot_tag_y,tag_yaw)   #对 Tag_id:1，若返回值为 True，则开始前往独木桥
                if (result == False):
                    continue

                base_action.action('Forwalk01')
                base_action.action('Forwalk01')
                base_action.action('Forwalk01')
                base_action.action('Forwalk01')
                base_action.action('Left3move')
                base_action.action('Left3move')
                base_action.action('Left3move')
                base_action.action('Left3move')
                base_action.action('Left3move')


            elif marker[0] == 2:
                stage = 'stage_bridge'
                result = turn_to_tag(robot_tag_x,robot_tag_y,tag_yaw)       #对 Tag_id:2，若返回值为 True，则开始过独木桥
                if (result == False):
                    continue
                print('行走至独木桥中间')
                base_action.action('fastForward03')

            elif marker[0] == 3:
                result = turn_to_tag(robot_tag_x,robot_tag_y,tag_yaw)       #对 Tag_id:3，若返回值为 True，则开始执行拆除易燃物
                if (result == False):
                    continue
                print('快走至独木桥终点')
                base_action.action('fastForward05')
                time.sleep(2)
                dismantler()
                time.sleep(0.5)
                base_action.action('HuanHu')                                              #当所有任务完成后，执行庆祝动作
                return

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()