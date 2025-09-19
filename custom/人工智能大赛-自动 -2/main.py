import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")

from leju import *



def main():
    nodes.node_initial()
    try:
        pass

    except Exception as e:
        nodes.serror(e)
        exit(2)
    finally:
        nodes.finishsend()
if __name__ == "__main__":
    print ("Run custom project")
    main()
