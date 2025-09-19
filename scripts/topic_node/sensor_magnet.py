import yaml
import os
import time
import math
import subprocess
import py_qmc5883l
import numpy as np

from magnet_calibration import Mag_Clibration
import threading

DEG_PER_RAD = 180.0/3.14159265358979
PATH = "/home/lemon/catkin_ws/src/aelos_smart_ros/scripts/topic_node/py_qmc5883l/mag.yaml"
QMC5883_ATTENTION_OFFSET = -90
restart_rosnode = "rosnode kill /sensor_service_node"

def is_connect_qmc5883l():
    out_str = bytes.decode(subprocess.check_output(['i2cdetect', '-y', '1']))
    return '0d' in out_str


class MagneticSensor:

    mag_calibrate = None

    def __init__(self):
        with open(PATH, encoding='utf-8') as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)

        self.mag_value_x = 0
        self.mag_value_y = 0
        self.mag_value_z = 0
        self.sensor = self.init_sensor()
        self.mag_values = 0
        self.mag_status = 0x03

        # 通过是否存在 clibration 值判断yaml版本 
        if 'yaml_version' in data.keys():
            print('mag.yaml 版本%d' % data['yaml_version'])
            self.yaml_version = 1

            mag_clibration = np.array(data['clibration'])
            self.sensor.calibration = mag_clibration

        else:
            print('mag.yaml 旧版,默认参数')
            self.yaml_version = 0

            self.x_offset = data["mag"]["x_offset"]
            self.y_offset = data["mag"]["y_offset"]
            self.x_scale = data["mag"]["x_scale"]
            self.y_scale = data["mag"]["y_scale"]
            self.x_min = 32700.0 
            self.x_max = -32700.0 
            self.y_min = 32700.0 
            self.y_max = -32700.0

        
    def init_sensor(self):
        if is_connect_qmc5883l():
            return py_qmc5883l.QMC5883L()
        # else:
        #     return Adafruit_LSM303.LSM303()
            
    def raw_magnet(self):
        if is_connect_qmc5883l():

            return 0,self.sensor.get_magnet_raw()
        else:
            return self.sensor.read()

    def wait_stop_mag_check(self):
        if self.mag_status == 0x02:
            try:
                self.mag_calibrate.stop_Calibration()
                time.sleep(0.5)
                calibration_matrix = self.mag_calibrate.calibration_info_to_yaml(PATH)
                
                self.sensor.calibration = calibration_matrix

                if self.yaml_version == 0:
                    try:
                        result = subprocess.check_output(restart_rosnode, shell=True)
                    except Exception as err:
                        print("err =",err)

                print("地磁校正结束")
                self.mag_status = 0x03
            except Exception as e:
                print("地磁校正结束错误,")
                print(e.args)
                self.mag_status = 0x04
        return self.mag_status


    def mag_check_timeout_fin(self):
        start_time = time.time()
        while True:
            now_time = time.time()
            if now_time - start_time > 8: # 16 - 6 -2
                self.wait_stop_mag_check()
                break

    def mag_check(self,is_func=False):
        if is_func:
            self.mag_status = 0x02
        print("地磁校正开始，请缓慢转动机器人")

        try:
            time.sleep(7)
        

            self.mag_calibrate = Mag_Clibration()

            threading.Thread(target=self.mag_check_timeout_fin).start()

            return_code = self.mag_calibrate.calibration(False)
            if return_code == self.mag_calibrate.CALIBRATION_FINISH:
                self.wait_stop_mag_check()
            elif return_code == self.mag_calibrate.CALIBRATION_STOP:
                self.mag_status = 0x04
                return self.mag_status

            self.mag_status = 0x03

        except Exception as e:
            self.mag_status = 0x04

        return self.mag_status

    def mag_check_calibrate(self,is_func=False):
        # start_time = time.time()
        if is_func:
            self.mag_status = 0x02
        print("地磁校正开始，请缓慢转动机器人")

        try:

            self.mag_calibrate = Mag_Clibration()

            return_code = self.mag_calibrate.calibration(False)
            if return_code == self.mag_calibrate.CALIBRATION_FINISH:
                self.wait_stop_mag_check()
            elif return_code == self.mag_calibrate.CALIBRATION_STOP:
                self.mag_status = 0x04
                return self.mag_status


            self.mag_status = 0x03

        except Exception as e:
            print('socket mag error:')
            print(e.args)
            self.mag_status = 0x04

        return self.mag_status

    def get_bearing(self):
        try:
            accel, mag = self.raw_magnet()
        except Exception as e:
            print(e.args)

        value_x, value_y, value_z = mag

        self.mag_value_x = value_x 
        self.mag_value_y = value_y 
        self.mag_value_z = value_z

        if self.yaml_version == 0:

            compass = math.atan2(-(value_y - self.y_offset) * self.y_scale,
                                (value_x - self.x_offset) * self.x_scale) * DEG_PER_RAD

            compass = compass - QMC5883_ATTENTION_OFFSET

            compass = compass - 360 if compass > 360 else compass
            compass = compass + 360 if compass < 0 else compass
            compass = int(math.floor(360 - compass))
            self.mag_values = compass
            return compass
        elif self.yaml_version == 1:
            self.mag_values = int(self.sensor.get_bearing())
            self.mag_values += QMC5883_ATTENTION_OFFSET
            if self.mag_values < 0:
                self.mag_values += 360
            elif self.mag_values >= 360:
                self.mag_values -= 360
            return self.mag_values

    def get_mag_status(self):
        return self.mag_status


    def get_mag_values(self):
        return self.mag_values

    def get_original_data(self):
        # Read data from magnetic and temperature data registers
        [x, y, z, t] = self.sensor.get_data()
        return x, y, z, t



if __name__ == '__main__':
    mag_sensor = MagneticSensor()
    # read = mag_sensor.mag_check()
    while True:
        read = mag_sensor.get_bearing()
        print("mag = ",read)
        time.sleep(0.5)

