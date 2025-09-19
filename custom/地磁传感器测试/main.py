import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")

from leju import *
import time

leju_variable_A = None



def main():
    nodes.node_initial()
    try:


        while 1 == 1:
            leju_variable_A = sensor_port.get_magnet()
            if leju_variable_A < 150:
                base_action.action('向右转动1步')
            if (leju_variable_A >= 150) and (leju_variable_A <= 210):
                base_action.action('向前慢走1步')
            if leju_variable_A > 210:
                base_action.action('向左转动1步')
            time.sleep(0.1)

    except Exception as e:
        nodes.serror(e)
        exit(2)
    finally:
        nodes.finishsend()
if __name__ == "__main__":
    print ("Run custom project")
    main()
