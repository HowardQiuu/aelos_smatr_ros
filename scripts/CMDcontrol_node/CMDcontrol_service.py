#!/usr/bin/env python3
#-*- coding:utf-8 -*-
############################
#File Name: server.py
#Author: Wang
#Mail: wang@hotmail.com
#Created Time:2017-09-25 15:17:38
############################

import rospy
from aelos_smart_ros.srv import *
from aelos_smart_ros.msg import sensor_topic

import cv2
import serial
import time
import binascii

import threading
import subprocess
import eventlet
import hashlib
import re
import json
import os

PACKAGE_HEAD = [0xFF, 0xFF]
RES_PACKAGE_HEAD = [0xFF, 0xFF]
PARAM_AMOUNT = 1
PARAM_LEN = 2
CHECK_BIT_LEN = 2
FLOAT_TYPE = 0x01
INT_TYPE = 0x02
STRING_TYPE = 0x03
FILTER_TIME = 0.2


runningAction = False
stopRunning = False
actionComplete = False      # activate() actionComplete = False
wifi_list = None

action_list = []
key_time = 0
Debug_flag = 0
control_key = 0
network_directory = "/home/lemon/NetworkManager"
ip_instruction = "ifconfig  {network_card_name} | head -n2 | grep inet | awk '{{print$2}}'"
################################################LUA函数请求#############################################

class timestamp():
    start_time = None

def set_wifi(wifi_name, wifi_password):
    connect_wifi_cmd = "sudo nmcli dev wifi connect '{name}' password '{password}'".format(name=wifi_name, password=wifi_password)
    try:
        result = subprocess.check_output(connect_wifi_cmd, shell=True)
        print(result)
        error_bytes = bytes("Error",encoding='utf8')
        if result.startswith(error_bytes):
            res = False
        else:
            if not os.path.exists(network_directory):
                os.makedirs(network_directory)
            data = {"name": wifi_name, "password": wifi_password}
            json_str = json.dumps(data, indent=4)
            with open("/home/lemon/NetworkManager/wifi_msg.json", "w") as f:
                f.write(json_str)
            res = True
    except Exception as err:
        print("err 37L")
        print(err)
        res = False

    return res


def handle_wifi(param, ser):
    wifi_list = "sudo iwlist wlan0 scan | grep ESSID"
    subprocess.check_output(wifi_list, shell=True)
    wifi_name = param[0]
    wifi_password = param[1]


    if set_wifi(wifi_name, wifi_password):
        result = [0x00, 0x00]
    else:
        result = [0x00, 0x01]
    # print(result)

    pac_len = [0x00, 0x07]
    pac_param_count = [0x01]
    pac_param = [0x03, INT_TYPE] + result

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)

    res = pac_data + pac_check_sum
    # print(res)
    ser.write(res)




# string to byte
def assemble_string(data):
    byte_param = []
    data_list = list(data)
    funname_data = list(map(lambda x: ord(x), data_list))
    return funname_data


def crc_calculate(package):
    crc = 0
    for hex_data in package:

        b2 = hex_data.to_bytes(1, byteorder='little')
        crc = binascii.crc_hqx(b2, crc)

    return [(crc >> 8), (crc & 255)]    # 校验位两位




def action_request(param, ser):
    global action_list,control_key,key_time

    if param != [0]:
        current_time = time.time()
        if current_time - key_time > FILTER_TIME:
            control_key = param
        key_time = current_time

    if len(action_list) != 0:
        if(Debug_flag):
            timestamp.start_time = time.time()
        action_name = action_list.pop(0)
    else:
        action_name = 'default'

    # action_name = 'music3'
    # print('REQUEST!')
    pac_data = assemble_string(action_name)
    total_len = PARAM_AMOUNT + PARAM_LEN + len(pac_data) + CHECK_BIT_LEN

    pac_param_len = [len(pac_data) + 1]
    pac_param_type = [STRING_TYPE]
    pac_param = pac_param_len + pac_param_type + pac_data

    pac_len = [( total_len>> 8), (total_len & 255)]
    pac_param_count = [0x01]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    
    # res_hex = map(lambda x: hex(x), res)
    # print(list(res_hex))
    ser.write(res)



def action_start(param, ser):
    global runningAction
    runningAction = True
    # print('start len(list):',len(action_list))

def action_complete(param, ser):
    global actionComplete
    actionComplete = True
    # print('complete')

def boardreceive_error(param, ser):
    # print('boardreceive_error')
    pass

def nametest(param, ser):
    print("nametest//////////")
    print(param)


def handle_mag(param, ser):
    try:
        rospy.wait_for_service('magnet_service')
        val = rospy.ServiceProxy('magnet_service', revise_mag)
        resp1 = val("revise")
        mag_status = resp1.success
    except rospy.ServiceException:
        mag_status = 0x04

        
    pac_len = [0x00, 0x07]
    pac_param_count = [0x01]

    pac_param = [0x03, INT_TYPE] + [0x00, mag_status & 0xff]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    ser.write(res)
    


def get_ip(param, ser):
    wlan0_ip = ip_instruction.format(network_card_name="wlan0")
    eth0_ip = ip_instruction.format(network_card_name="eth0")
    wlan0_result = str.strip(subprocess.check_output(wlan0_ip, shell=True).decode())
    eth0_result = str.strip(subprocess.check_output(eth0_ip, shell=True).decode())

    if wlan0_result != "" and eth0_result != "":
        ip = wlan0_result + "," + eth0_result
    elif wlan0_result == "":
        ip = eth0_result
    else:
        ip = wlan0_result

    
    if ip == "":
        ip = 'None'

    pac_data = assemble_string(ip)
    total_len = PARAM_AMOUNT + PARAM_LEN + len(pac_data) + CHECK_BIT_LEN

    pac_param_len = [len(pac_data) + 1]
    pac_param_type = [STRING_TYPE]
    pac_param = pac_param_len + pac_param_type + pac_data

    pac_len = [( total_len>> 8), (total_len & 255)]
    pac_param_count = [0x01]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    ser.write(res)



def scan_wifi(param, ser):
    global wifi_list
    if param[0] == 0:
        wifi_name_list = []
        wifi = "sudo nmcli -f SSID,BSSID,SSID-HEX,SIGNAL,SECURITY,ACTIVE device wifi"
        result = subprocess.check_output(wifi, shell=True)
        result = result.decode()
        partern = r'\n((?:\S.*\S)|(?: )) +(\w\w(?::\w\w){5})\s*[A-F0-9]+ +(\d+)\s*(\-{2}|\w+.?[\w|\.]+)\s*(\w+)'
        active_wifi_list = re.findall(partern, result)
        for wifi in active_wifi_list:
            wifi_name_list.append(wifi[0])

        wifi_list = (str(wifi_name_list)).replace("'",'"')
    send_wifi = wifi_list[param[0]:param[1]]

    pac_data = assemble_string(send_wifi)
    total_len = PARAM_AMOUNT + PARAM_LEN + len(pac_data) + CHECK_BIT_LEN

    pac_param_len = [len(pac_data) + 1]
    pac_param_type = [STRING_TYPE]
    pac_param = pac_param_len + pac_param_type + pac_data

    pac_len = [( total_len>> 8), (total_len & 255)]
    pac_param_count = [0x01]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    ser.write(res)

def Reply_SensorStatus(param, ser):
    try:
        sensor_msg = rospy.wait_for_message('/sensor_info', sensor_topic)

        ADC_1 = [(sensor_msg.IO_1 >> 8 & 0xff), (sensor_msg.IO_1 & 0xff)]
        ADC_2 = [(sensor_msg.IO_2 >> 8 & 0xff), (sensor_msg.IO_2 & 0xff)]
        magnet = [(sensor_msg.mag >> 8 & 0xff), (sensor_msg.mag & 0xff)]
        magnet_x = [(sensor_msg.mag_x >> 8 & 0xff), (sensor_msg.mag_x & 0xff)]
        magnet_y = [(sensor_msg.mag_y >> 8 & 0xff), (sensor_msg.mag_y & 0xff)]
        magnet_z = [(sensor_msg.mag_z >> 8 & 0xff), (sensor_msg.mag_z & 0xff)]

        package_len = [0x00, 0x1B]
        package_param_count = [0x06]

        ADC1_param = [0x03, 0x02] + ADC_1
        ADC2_param = [0x03, 0x02] + ADC_2
        magnet_param = [0x03, 0x02] + magnet
        magnet_param_x = [0x03, 0x02] + magnet_x
        magnet_param_y = [0x03, 0x02] + magnet_y
        magnet_param_z = [0x03, 0x02] + magnet_z

        package_data = RES_PACKAGE_HEAD + package_len + package_param_count + ADC1_param + ADC2_param + magnet_param + magnet_param_x + magnet_param_y + magnet_param_z
        package_check_sum = crc_calculate(package_data)
        res = package_data + package_check_sum
        ser.write(res)
    except Exception as e:
        print(e)

def scan_wifi(param, ser):
    global wifi_list
    if param[0] == 0:
        wifi = "sudo iwlist wlan0 scan | grep ESSID"
        result = subprocess.check_output(wifi, shell=True)
        wifi_list = "[" + result.decode().replace(' ', '').replace('\n', '').replace('ESSID', '').replace(':', ',')[1:] + "]"
    send_wifi = wifi_list[param[0]:param[1]]

    pac_data = assemble_string(send_wifi)
    total_len = 1 + 2 + len(pac_data) + 2

    pac_param_len = [len(pac_data) + 1]
    pac_param_type = [0x03]
    pac_param = pac_param_len + pac_param_type + pac_data

    pac_len = [( total_len>> 8), (total_len & 255)]
    pac_param_count = [0x01]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    ser.write(res)

FUNC_MAP = {
    "REQ": action_request,
    "START": action_start,
    "COMPLETE": action_complete,
    "ERROR": boardreceive_error,
    "REtest": nametest,
    "wificonnect": handle_wifi,
    "magcalib": handle_mag,
    "gainip": get_ip,
    "SensorInput": Reply_SensorStatus,
    "scanwifi":scan_wifi

}
################################################函数#############################################


def action_wait(wait_time):
    global runningAction, actionComplete
    # print('action wait ') #fftest
    if actionComplete == False:
        eventlet.monkey_patch(time=True)
        with eventlet.Timeout(wait_time,False):
            while len(action_list) != 0 or actionComplete == False:
                # sys.stdout.flush()
                rospy.sleep(0.001)
                
    print("action_wait done")
    runningAction = False



def parse_float(data, data_length):
    float_length = 4

    if data_length == float_length:
        # single float param
        float_string = ''
        for byte in data:
            float_string = float_string + byte

        return struct.unpack('f', float_string)[0]
    else:
        # float array param
        current_index = 0
        float_array = []

        while current_index < data_length:
            current_data = data[current_index:current_index+float_length]
            current_float = parse_float(current_data, float_length)

            float_array.append(current_float)
            current_index = current_index + float_length

        return float_array

def parse_int(data, data_length):
    int_length = 2

    if data_length == int_length:
        # single int param
        return (data[0] << 8) + data[1]
    else:
        #int array param
        current_index = 0
        int_array = []

        while current_index < data_length:
            current_data = data[current_index:current_index+int_length]
            current_int = parse_int(current_data, int_length)

            int_array.append(current_int)
            current_index = current_index + int_length

        return int_array

def parse_string(data):
    string_param = ''

    for byte in data:
        string_param = string_param + chr(byte)

    return string_param

def parse_cmd(cmd_data, ser):
    # process request name, ignore \0
    cmd_name_len = cmd_data[0]
    cmd_name_data = cmd_data[1:cmd_name_len]
    cmd_name = parse_string(cmd_name_data)
    # print("cmd name is: ", cmd_name)

    # process parameters
    cmd_param = []
    cmd_param_count = cmd_data[cmd_name_len+1]

    current_count = 0
    current_index = cmd_name_len + 2
    while current_count < cmd_param_count:
        current_param_len = cmd_data[current_index]
        current_param_type = cmd_data[current_index+1]
        if current_param_type == FLOAT_TYPE:
            # float param
            current_param_data = recv[current_index+2:current_index+current_param_len+1]
            current_param = parse_float(current_param_data, current_param_len-1)
        elif current_param_type == INT_TYPE:
            # int param
            current_param_data = cmd_data[current_index+2:current_index+current_param_len+1]
            current_param = parse_int(current_param_data, current_param_len-1)
        else:
            # string param
            current_param_data = cmd_data[current_index+2:current_index+current_param_len+1]
            current_param = parse_string(current_param_data)

        cmd_param.append(current_param)
        current_count += 1
        current_index = current_index + current_param_len + 1
    # print("CMD are: ", cmd_name)

    process_func = FUNC_MAP.get(cmd_name)
    process_func(cmd_param, ser)


def main():
    tmp_head = []
    with serial.Serial('/dev/serial0', 9600) as ser:
        print("提示...")
        
        while not rospy.is_shutdown():
            cur_byte = ser.read()
            # print("ser???")
            tmp_head.append(ord(cur_byte))
            # print(tmp_head)
            if len(tmp_head) == 2:
                if tmp_head == PACKAGE_HEAD:
                    if(Debug_flag):
                        print("receive data:")
                    length_high = ord(ser.read())
                    length_low = ord(ser.read())
                    package_data_length = (length_high << 8) + length_low
                    package_data_byte = ser.read(package_data_length)
                    package_data = list( package_data_byte)

                    # crc check
                    tmp_package = PACKAGE_HEAD + [length_high, length_low] + package_data[0:len(package_data)-2]
                    package_crc_sum = package_data[len(package_data)-2:]
                    tmp_crc_sum = crc_calculate(tmp_package)
                    if(Debug_flag):
                        print(tmp_package)  # [255, 255, 0, 12, 4, 82, 69, 81, 0, 1, 3, 2, 0, 0]
                        # print(tmp_crc_sum)#fftest

                    if package_crc_sum == tmp_crc_sum:
                        try:
                            parse_cmd(package_data, ser)
                        except Exception as err:
                            data_hex = map(lambda x: hex(x), package_data)
                            print(list(data_hex))
                            print("CMD_transfer():error")
                    else:
                        print("CMD_transfer():Incorrect checksum!")

                    tmp_head = []
                else:
                    tmp_head.pop(0)
                # main()


acted_name = ""
def action_append(act_name, wait_time):
    global acted_name , action_list, actionComplete
    
    m = hashlib.md5()
    name_encode = bytes(act_name,encoding='utf8')
    m.update(name_encode)
    acted_name = 'leju_' + m.hexdigest()
    print(acted_name)#fftest

    actionComplete = False
    if len(action_list) > 0 :
        print("队列超过一个动作")

    action_list.append(acted_name)

    if(Debug_flag):
        print("执行动作名：",act_name)
        timestamp.start_time = time.time()
        start_time = timestamp.start_time

        action_wait(wait_time)

        if start_time == timestamp.start_time:
            print("通讯异常,请查阅文档排查情况：https://www.lejuhub.com") #目前暂时用此地址代替，待完善文档后需将此地址修改为对应的文档地址

    else:
        action_wait(wait_time)


def action_receive(req):
    act_name = ""
    # print ("%s"%req.data)
    act_name = "%s"%req.data
    if len(action_list) == 0:
        if act_name != None:
            # print("act_name")
            action_append(act_name,req.wait_time)
    return [True]


def remote_control_key(req):
    global control_key
    if "%s"%req.data == "KEY":
        get_key = control_key
        control_key = 0
    return get_key

def empty_keys(req):
    global control_key
    if "%s"%req.data == "KEY":
        control_key = 0
    return [True]

def init_service():
    rospy.Service('action_receive', CMDcontrol, action_receive)
    rospy.Service('remote_control_key', get_key, remote_control_key)
    rospy.Service('empty_keys', empty_key, empty_keys)
    # rospy.spin()

    
if __name__ == '__main__':
    rospy.init_node('action_server', anonymous = True)
    init_service()
    main()
