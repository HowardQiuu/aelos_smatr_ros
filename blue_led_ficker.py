import RPi.GPIO as GPIO
import threading
import time
import subprocess
import xml.etree.ElementTree as ET

launch_file_path = "/home/lemon/catkin_ws/src/aelos_smart_ros/launch/ar_track.launch"
TAG_NODE = "node"
led_sign = True
LED_CTRL_PORT = 17
TIME_LAPSE = 1

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)

GPIO.setup(LED_CTRL_PORT,GPIO.OUT)

def rosnode_quantity():
    cmd = "rosnode list"
    rosnode_list = ""
    try:
        result = subprocess.check_output(cmd, shell=True).decode().replace('\n', ',').replace('/head/', '').replace('/chest/', '').replace('/', '')[:-1]
        rosnode_list = result.split(",")
        
    except Exception as err:
        print(err)

    return rosnode_list


def led_ficker():
    while True:
        if led_sign:
            GPIO.output(LED_CTRL_PORT,GPIO.HIGH)
            time.sleep(TIME_LAPSE)
            GPIO.output(LED_CTRL_PORT,GPIO.LOW)
            time.sleep(TIME_LAPSE)
        else:
            break


if __name__ == "__main__":
    th1 = threading.Thread(target=led_ficker)
    th1.setDaemon(True)
    th1.start()

    roslaunch_list = []

    tree = ET.parse(launch_file_path)
    root = tree.getroot()
    for child in root:
        if child.tag == TAG_NODE:
            roslaunch_list.append(child.attrib["name"])


    while True:
        rosnode_list = rosnode_quantity()
        if set(roslaunch_list) < set(rosnode_list):
            led_sign = False
            GPIO.output(LED_CTRL_PORT,GPIO.LOW)
            break
        else:
            continue
        time.sleep(0.5)
        




