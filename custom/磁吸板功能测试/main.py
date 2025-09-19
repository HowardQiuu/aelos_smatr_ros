import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")

from leju import *
import time



def main():
    nodes.node_initial()
    try:


        while 0 == 0:
            sensor_port.set_output(1, 1)
            time.sleep(2)
            sensor_port.set_output(1, 0)
            time.sleep(2)

    except Exception as e:
        nodes.serror(e)
        exit(2)
    finally:
        nodes.finishsend()
if __name__ == "__main__":
    print ("Run custom project")
    main()
