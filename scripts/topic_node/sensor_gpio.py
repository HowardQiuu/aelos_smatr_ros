#!/usr/bin/env python
# -*- coding: utf-8 -*-



import RPi.GPIO as GPIO
import Adafruit_ADS1x15

import time
import threading

# port_num: 端口号 1 2

class GPIOModule:
	def __init__(self):
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(16, GPIO.IN)
		GPIO.setup(20, GPIO.IN)

		self.adc = Adafruit_ADS1x15.ADS1015()
		self.adc_data = [0, 0]

		# read adc data every 100 ms
		self.time_interval = 0.1

        # 0 for input, 1 for output
		self.gpio_status = {
			16: 0,
			20: 0
		}

		self.gpio_level = {
			16:0,
			20:0
		}

		self.pwm_list = {
		    16: None,
		    20: None
		}

		self.port_map = {
		    0x01: 20,
		    0x02: 16
		}

		self.adcport_map = {
		    0x01: 2,
		    0x02: 1
		}

	def change_to_input(self, port_num):
		if self.pwm_list[port_num] != None:
			self.pwm_list[port_num].stop()
			self.pwm_list[port_num] = None
		GPIO.setup(port_num, GPIO.IN)
		self.gpio_status[port_num] = 0


	def change_to_output(self, port_num):
		GPIO.setup(port_num, GPIO.OUT)
		self.pwm_list[port_num] = GPIO.PWM(port_num, 100)
		self.pwm_list[port_num].start(0)
		self.gpio_status[port_num] = 1


	def set_gpio_io(self, port_num, status):
		if self.gpio_status.get(port_num) != status:
			if status == 1:
				self.change_to_output(port_num)
			else:
				self.change_to_input(port_num)

			# add a delay after change I/O status
			time.sleep(self.time_interval)


	def write_gpio(self, port_num, vol):
		target_pwm = self.pwm_list.get(port_num)

		if vol > 0:
			self.gpio_level[port_num] = 1
			target_pwm.ChangeDutyCycle(100)
		else:
			self.gpio_level[port_num] = 0
			target_pwm.ChangeDutyCycle(0)
			

	def set_vol(self, port_num, vol):
		gpio_port = self.port_map.get(port_num)
		self.set_gpio_io(gpio_port, 1)
		self.write_gpio(gpio_port, vol)


	def read_adc_io(self):   #给0v的时候为0，5v的时候为209
		self.change_to_input(self.port_map.get(1))
		self.change_to_input(self.port_map.get(2))
		while True:
			time.sleep(self.time_interval)

			adc0 = int(self.adc.read_adc(2, gain=1)/8)
			adc1 = int(self.adc.read_adc(1, gain=1)/8)
			adc2 = int(self.adc.read_adc(3, gain=1)/8)

			# to keep gain value same with edu robot, divided by 1.22
			adc0 = int(adc0 / 1.22)
			adc1 = int(adc1 / 1.22)
			adc2 = int(adc2 / 1.22)

			adc0 = 0 if adc0 < 0 else adc0
			adc1 = 0 if adc1 < 0 else adc1
			adc2 = 0 if adc2 < 0 else adc2
			self.adc_data = [adc0, adc1]
			return self.adc_data

	def read_io(self,port_num):   #给0v的时候为0，5v的时候为209 --1 2
			# self.change_to_input(self.port_map.get(port_num))
			# while True:
			time.sleep(self.time_interval)

			adc = int(self.adc.read_adc(self.adcport_map.get(port_num), gain=1)/8)

			adc = 0 if adc < 0 else adc
			self.adc_data = adc
			return self.adc_data

	def read_all_vol(self):
		return self.adc_data


	def read_vol(self, port_num):
		gpio_port = self.port_map.get(port_num)
		self.set_gpio_io(gpio_port, 0)

		return self.adc_data[port_num-1]


	def start_gpio(self):
		threading.Thread(target=self.read_adc_io).start()

	def get_gpio_status(self,io):
		return self.gpio_status[self.port_map.get(io)]

	def get_gpio_level(self,io):
		return self.gpio_level[self.port_map.get(io)]




if __name__ == '__main__':
    PINset = GPIOModule()
    while True:
        print("set low")
        PINset.set_vol(1,1)
        PINset.set_vol(2,1)
        time.sleep(5)

        print("set high")
        PINset.set_vol(2,0)
        PINset.set_vol(1,0)
        time.sleep(5)
