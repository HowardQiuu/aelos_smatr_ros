import sys
sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")

from leju import *

leju_variable_key = None



def main():
    nodes.node_initial()
    try:


        while 0 == 0:
            leju_variable_key = get_key.key()
            if leju_variable_key == 193:
                base_action.action('抱起方块1')
            if leju_variable_key == 195:
                base_action.action('放下方块1')
            if leju_variable_key == 196:
                base_action.action('上楼梯10')
                base_action.action('起身4')
            if leju_variable_key == 194:
                base_action.action('抱箱右移1')
            if leju_variable_key == 198:
                base_action.action('抱箱前进1')
            if leju_variable_key == 199:
                base_action.action('抱箱后退1')
            if leju_variable_key == 197:
                base_action.action('包厢左转加速版')
            if leju_variable_key == 200:
                base_action.action('包厢右转加速版')
            if leju_variable_key == 243:
                base_action.action('前倒地')
            if leju_variable_key == 245:
                base_action.action('后倒地')
            if leju_variable_key == 201:
                base_action.action('上台阶')
            if leju_variable_key == 203:
                base_action.action('翻墙稳定1')
            if leju_variable_key == 205:
                base_action.action('抱起10cm方块')
            if leju_variable_key == 202:
                base_action.action('放下10cm方块')
            if leju_variable_key == 207:
                base_action.action('抱10cm方块前进')
            if leju_variable_key == 220:
                base_action.action('抱10cm方块后退')
            if leju_variable_key == 206:
                base_action.action('抱10cm方块左移')
            if leju_variable_key == 222:
                base_action.action('抱10cm方块右移')
            if leju_variable_key == 246:
                base_action.action('前翻滚')
            if leju_variable_key == 244:
                base_action.action('加速1步上楼梯第二版')

    except Exception as e:
        nodes.serror(e)
        exit(2)
    finally:
        nodes.finishsend()
if __name__ == "__main__":
    print ("Run custom project")
    main()
