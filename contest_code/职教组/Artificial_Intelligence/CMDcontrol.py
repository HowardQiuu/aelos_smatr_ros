#!/usr/bin/env python3
# coding:utf-8

import time
import rospy
import threading
import action_task
import turn_over_barrier
import flammable
import box
import sys

import os

sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import *


TIME_SKIP_AFTER_ACTION = 1.5
RATE_SCAN_KEY_TIME = 0.5

class CMDcontrol:

    now_key = 0
    key = 0
    act_fin_time = 0

    def scan_key(self):
        while not rospy.is_shutdown():
            self.now_key = get_key.key()
            if self.now_key != 0 and self.key != self.now_key:
                delta_time = time.time() - self.act_fin_time
                if delta_time < TIME_SKIP_AFTER_ACTION:
                    continue
                self.key = self.now_key
            time.sleep(RATE_SCAN_KEY_TIME)


    def get_key(self):
        return self.key


    def action_append(self,act_name):
        base_action.action(act_name)


def main():
    CMD = CMDcontrol()
    keyScanner_thread = threading.Thread(target=CMD.scan_key, daemon=True)
    keyScanner_thread.start()


    while not rospy.is_shutdown():
        try:
            key = CMD.get_key()
            if key != 0:
                if key == 197:                                  # X
                    #组合任务
                    action_task.main(CMD)
                elif key == 198:                                # Y
                    #拆除易燃物
                    flammable.dismantler(CMD)
                elif key == 199:                                # A
                    #人脸识别
                    face_identification.main(CMD)
                elif key == 200:                                # B
                    #抱箱子
                    box.main(CMD)
                elif key == 244:                                # RB
                    #翻越障碍
                    turn_over_barrier.barrier(CMD)

                CMD.act_fin_time = time.time()
                CMD.key = 0
            time.sleep(RATE_SCAN_KEY_TIME)
        except Exception as e:
            print(e)

        

if __name__ == '__main__':
    rospy.init_node('CMDcontrol') 

    print("正在检测tflite环境, 请耐心等待手柄控制程序启动")
    tflite_env = os.popen('pip freeze | grep tflite-support').read()
    if tflite_env == '':
        print("正在安装环境，请稍后...")
        os.system('./age_gender/setup.sh')
        print("安装已完成")
    else:
        print(tflite_env + "环境已安装")

    import face_identification
    print("手柄控制程序已启动")

    main()

