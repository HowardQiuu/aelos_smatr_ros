#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ********************
# Author : z2h
# Last Modification : 
# Comment : 
#   websocket连接结点
#   主要功能：
#       1. 开启ws服务监听客户端信息,
# ********************

import os
import time
import rospy
import asyncio
import websockets

from aelos_smart_ros.msg import *
from aelos_smart_ros.srv import *
from websockets.legacy.server import WebSocketServerProtocol
from std_msgs.msg import String
from multiprocessing import Process
from multiprocessing import Pipe
import subprocess
import json
import threading
from pathlib import Path
import subprocess
import psutil


# 正经的全局配置
ip_instruction = "ifconfig  {network_card_name} | head -n2 | grep inet | awk '{{print$2}}'"
chest_image_url = "http://{ip}:8080/stream_viewer?topic=/usb_cam_chest/image_raw"
head_image_url = "http://{ip}:8080/stream_viewer?topic=/usb_cam_head/image_raw"
WS_PUBLISHRATE = 10     # publish速率
LOCAL_HOST = '0.0.0.0'  # 接收所有ip
PORT = 9200
PROJECT_PREFIX = "/home/lemon/catkin_ws/src/aelos_smart_ros/custom/"
COMMAND_DICT = {                # 桌面软件指令与执行命令的对应关系, val对应类中的处理函数
    "mkdir" : "create_dir",
    "run" : "run_project",
    "stop": "stop_main",
    "start_calibrate_magnet": "start_calibrate_magnet",
    "stop_calibrate_magnet": "stop_calibrate_magnet",
    "get_magnet_status": "get_magnet_status",
    "get_robot_status":"get_robot_status"
}

LOG_FMT = "[{}] {}"
DEBUG_FMT = "{:>5} {} {}"
DEBUG = 1
Err = 3



class Aelos_websocket(object):
    def __init__(self):
        self.ws_obj = None


    def __new__(cls, *args, **kwargs):
        if not hasattr(Aelos_websocket, "_instance" ):
            Aelos_websocket._instance = object.__new__(cls)
        return Aelos_websocket._instance
        

    def main(self):
        """开启服务
        """
        ws_handler = publish_content
        
        ws_config = {
            "ws_handler" : ws_handler,
            "host" : LOCAL_HOST,                 # 测试地址，如果需要做接口转发的话本地配个nginx
            "port" : PORT,
            "create_protocol" : SingleWebSocketServeProtol
        }
        self.ws_obj = websockets.serve(**ws_config)
        asyncio.get_event_loop().run_until_complete(self.ws_obj)
        asyncio.get_event_loop().run_forever()


    async def send(self,s,i):
    # 发送程序执行情况
        try:
            if len(list(self.ws_obj.ws_server.websockets)):
                ws = list(self.ws_obj.ws_server.websockets)[i]
                await ws.send(s)
            else:
                print("当前无连接对象:",len(list(self.ws_obj.ws_server.websockets)))
                return
        except Exception as e:
            print(e)


    def start_send(self,s,i=0):
        # 发送消息线程
        task = [self.send(s,i)]
        loop1 = asyncio.new_event_loop() 
        asyncio.set_event_loop(loop1)
        asyncio.get_event_loop().run_until_complete(asyncio.wait(task))




# ****************************************
#                 ws消息处理
# ****************************************

class WsHandler(object):
    """解析并响应websocket指令
    """

    def __init__(self):
        self.have_multi_params = lambda x: True if ',' in x else False          # 检查一下是不是多参数监查起
        self.have_cmd = lambda x: True if x in COMMAND_DICT else False          # 检查是否有解析命令
        self.cmd = None
        self.reply = None
        self.cmd_handler = None
        self.msg = None
        self.run_res = None
        self.stop_res = None


    def __call__(self, client_msg):
        return self.handle_message(client_msg)


    def handle_message(self, client_msg):
        """客户端消息处理主逻辑
        """

        self.msg = json.loads(client_msg)     #处理json数据
        self.cmd = self.msg['cmd']

        if not self.have_cmd(self.msg['cmd']): return "Unkown command"              # 没有命令的操作函数，返回

        self.cmd_handler = self.__getattribute__(COMMAND_DICT.get(self.cmd))
        th2 = threading.Thread(target=self.cmd_handler,args=(self.msg.get('data',{}),))
        th2.start()                         # 执行命令
        if DEBUG == 1: print (DEBUG_FMT.format("", "run cmd : " + self.cmd, "res : {}".format(self.run_res)))

        return self.run_res


    # ********** 指令处理函数 **********
    # 添加新功能在这里进行扩充
    # 注：
    #   - 新增加指令处理函数需要同步在COMMAND_DICT进行更新
    #   - 处理结果直接作为运行状态返回
    # ********************************
    def create_dir(self, dir_path):
        """创建文件
        """
        try:
            dir_path = PROJECT_PREFIX + dir_path
            os.makedirs(dir_path, exist_ok=True)
            return True
        except:
            return False


    def run_project(self, data):
        """运行项目
        """
        try:
            main_proc = check_program_running()
            if main_proc:
                main_proc.terminate()
                
            project_name = data['project_name']

            my_file = Path("{}/main.py".format(PROJECT_PREFIX+project_name))         #判断文件或目录是否存在
            if my_file.is_file():
                my_file = subprocess.list2cmdline([my_file])
                cmd = "python {}".format(my_file)
                reply = {"cmd":self.cmd,"code":0,"runState":0}
                reply = json.dumps(reply,ensure_ascii=False)
                wes.start_send(reply)
            else:
                reply = {"cmd":self.cmd,"code":Err,"runState":0,"msg":"No files or directories"}
                reply = json.dumps(reply,ensure_ascii=False)
                wes.start_send(reply)
                return

            self.run_res = subprocess.run(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")

            if self.cmd != "stop":
                reply = {"cmd":self.cmd,"code":self.run_res.returncode,"runState":1,"stdout":self.run_res.stdout,"stderr":self.run_res.stderr}
                reply = json.dumps(reply,ensure_ascii=False)
                wes.start_send(reply)


            self.run_res = None
        except Exception as e:
            return False
        return

    def stop_main(self, data):
        """终止运行
        """
        try:
            cmd = "ps -ef | grep main.py | grep -v grep | awk '{print $2}' | xargs kill -9"
            self.stop_res = subprocess.run(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding="utf-8")
            reply = {"cmd":self.cmd,"code":self.stop_res.returncode,"stdout":self.stop_res.stdout,"stderr":self.stop_res.stderr}
            reply = json.dumps(reply,ensure_ascii=False)
            wes.start_send(reply)
            self.stop_res = None
        except Exception as e:
            return False
        return 

    def start_calibrate_magnet(self, data):
        try:
            rospy.wait_for_service('magnet_service')
            val = rospy.ServiceProxy('magnet_service', revise_mag)
            resp1 = val("start_calibrate")
            mag_status = resp1.success

            reply = {"cmd":self.cmd,"mag_status":mag_status}
            reply = json.dumps(reply,ensure_ascii=False)

            wes.start_send(reply)
        except Exception as e:
            mag_status = 0x04

            reply = {"cmd":self.cmd,"mag_status":mag_status}
            reply = json.dumps(reply,ensure_ascii=False)

            wes.start_send(reply)
            return False
        return True

    def stop_calibrate_magnet(self, data):
        try:
            rospy.wait_for_service('magnet_service')
            val = rospy.ServiceProxy('magnet_service', revise_mag)
            resp1 = val("stop_calibrate")
            mag_status = resp1.success

            reply = {"cmd":self.cmd,"mag_status":mag_status}
            reply = json.dumps(reply,ensure_ascii=False)

            wes.start_send(reply)
        except Exception as e:
            return False
        
    def get_magnet_status(self, data):
        try:
            rospy.wait_for_service('magnet_service')
            val = rospy.ServiceProxy('magnet_service', revise_mag)
            resp1 = val("get_mag_status")
            mag_status = resp1.success

            reply = {"cmd":self.cmd,"mag_status":mag_status}
            reply = json.dumps(reply,ensure_ascii=False)

            wes.start_send(reply)
        except Exception as e:
            return False
        
    def get_robot_status(self, data):
        try:
            wlan0_ip = ip_instruction.format(network_card_name="wlan0")
            eth0_ip = ip_instruction.format(network_card_name="eth0")
            wlan0_result = str.strip(subprocess.check_output(wlan0_ip, shell=True).decode())
            eth0_result = str.strip(subprocess.check_output(eth0_ip, shell=True).decode())
            if wlan0_result != "":
                robot_ip = wlan0_result
            else:
                robot_ip = eth0_result
            
            chest_image = chest_image_url.format(ip=robot_ip)
            head_image = head_image_url.format(ip=robot_ip)

            reply = {"cmd":self.cmd,"msg":{"ip":robot_ip,"chest_image":chest_image,"head_image":head_image}}
            reply = json.dumps(reply,ensure_ascii=False)

            wes.start_send(reply)
        except Exception as e:
            print(e)
    
# ****************************************
#                 连接管理
# ****************************************

class SingleWebSocketServeProtol(WebSocketServerProtocol):
    """ws服务器中的连接对象

    """
    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        """新连接的注册函数

        直接加一个逻辑对目前已经存在的服务数量进行判断，如果已经存在连接，就忽略新的连接
        此处还可以进行扩充进行链接管理维护：例如维护多个数量链接，或者链接调度
        """
        print ("[{}] Receive new connection".format(time.asctime()))
        print ("{:>5}Existing connections : {}".format("", len(self.ws_server.websockets)))
        
        if self.ws_server.websockets: 
            print ("{:>5}Igore this connection".format(""))
            return

        print ("{:>5}Create new connection".format(""))
        super().connection_made(transport)
        self.ws_server.register(self)
        main_proc = check_program_running()
        if main_proc:
            main_proc.terminate()

        check_magnet_status()
        # self.handler_task = self.loop.create_task(self.handler())     # 这一行是原有处理逻辑里的，不知道有什么用，但是不注释会报错


# ****************************************
#           websocket服务器初始化
# ****************************************


async def publish_content(ws, path):
    """ ws服务器响应函数

    Parameters
    ----------
    ws : WebSocketServerProtocol 
        客户端连接对象
    path : 
        为接口自带参数
    """
    ws_handler = WsHandler()
    async for msg in ws:
        print (LOG_FMT.format(time.asctime(), "Recive message :" + msg))
        res = ws_handler(msg)



# ****************************************
#           service获取打印信息
# ****************************************

def send_websocket_msg(req):
    msg = "%s"%req.data
    reply = {"cmd":"leju_log","msg":msg}
    reply = json.dumps(reply,ensure_ascii=False)
    wes.start_send(reply)
    return [True]


def check_program_running():
    current_pid = psutil.Process().pid
    
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'python' and proc.pid != current_pid:
            try:
                cmdline = proc.cmdline()
                if len(cmdline) > 1 and cmdline[1].endswith('main.py'):
                    return proc
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue
    
    return None


def check_magnet_status():
    try:
        rospy.wait_for_service('magnet_service')
        val = rospy.ServiceProxy('magnet_service', revise_mag)
        resp1 = val("get_mag_status")
        mag_status = resp1.success
        if mag_status == 0x02:
            stop_magnet()
    except Exception as e:
        return False

def stop_magnet():
    try:
        rospy.wait_for_service('magnet_service')
        val = rospy.ServiceProxy('magnet_service', revise_mag)
        resp1 = val("stop_calibrate")
    except Exception as e:
        print(e)


def init_service():
    rospy.Service('send_websocket_msg', information, send_websocket_msg)
    # rospy.spin()



if __name__ == "__main__":
    rospy.init_node('ws_talker', anonymous=True)           # 初始化节点

    init_service()
    wes = Aelos_websocket()
    wes.main()