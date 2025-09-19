import RPi.GPIO as GPIO
import time
import signal
import sys

LED_CTRL_PORT = 17
TIME_LAPSE = 0.1

GPIO.setwarnings(False)
# 设置GPIO模式为BCM
GPIO.setmode(GPIO.BCM)

GPIO.setup(LED_CTRL_PORT, GPIO.OUT)

def blink_led():
    while True:
        GPIO.output(LED_CTRL_PORT, GPIO.HIGH) 
        time.sleep(TIME_LAPSE) 
        GPIO.output(LED_CTRL_PORT, GPIO.LOW)
        time.sleep(TIME_LAPSE) 

def signal_handler(signal, frame):
    GPIO.output(LED_CTRL_PORT, GPIO.LOW)
    GPIO.cleanup()
    sys.exit(0)


if __name__ == '__main__':
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)

    # 启动LED闪烁
    blink_led()
