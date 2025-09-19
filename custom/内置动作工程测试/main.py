import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")

from leju import *
import time

leju_variable_A = None



def main():
    nodes.node_initial()
    try:


        while 0 == 0:
            leju_variable_A = get_key.key()
            if leju_variable_A == 193:
                music.music_play('JG')
                base_action.action('背手鞠躬')
            if leju_variable_A == 196:
                music.music_play('DL')
                base_action.action('俯卧撑')
            if leju_variable_A == 195:
                music.music_play('DPZC')
                base_action.action('大鹏展翅')
            if leju_variable_A == 194:
                music.music_play('ZJSM')
                base_action.action('左脚射门')
            if leju_variable_A == 197:
                music.music_play('江南style')
                base_action.action('江南style')
            if leju_variable_A == 198:
                music.music_play('勃拉姆斯')
                base_action.action('勃拉姆斯')
            time.sleep(0.1)

    except Exception as e:
        nodes.serror(e)
        exit(2)
    finally:
        nodes.finishsend()
if __name__ == "__main__":
    print ("Run custom project")
    main()
