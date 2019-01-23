"""
This python module is used to positioning in indoor environment based on UWB and IMU
"""
import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import threading
import time
import math
import operator
import socket
import os
import numpy as np
from pyquaternion import Quaternion
from numpy import linalg as LA
import EKF_6states as EKF6
from Queue import Queue

acc = np.zeros([1,3])
gro = np.zeros([1,3])
mag = np.zeros([1,3])

# Initialize EKF 6-states parameters
ekf6 = EKF6.EKF_6states(0.01)

# IMU Initialization
SETTINGS_FILE = "RTIMULib"
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
if (not imu.IMUInit()):
	print ("IMU Initialize Failed.")
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

# IMU Thread
class Get_IMU_Data(threading, Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		while True:
			if imu.IMURead():
				data = imu.getIMUData()
				acc = data["accel"]
				gro = data["gyro"]
				mag = data["compass"]
			print ("task-IMU")
			time.sleep(0.01)

# DWM Thread
class Get_UWB_Data(threading, Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		while True:
			print ("task-UWB")
			time.sleep(0.01)

# 6-states EKF thread
class EKF_Cal_Euler(threading, Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		while True:
			print ("task-Euler")
			time.sleep(0.01)

# main Thread
def main():
	queue = Queue()
	imu = Get_IMU_Data('IMU.', queue)
	uwb = Get_UWB_Data('UWB.', queue)
	euler = EKF_Cal_Euler('Euler.',queue)
	imu.start()
	uwb.start()
	euler.start()
	imu.join()
	uwb.join()
	euler.join()
	print ('All threads terminate!')


if __name__ == '__main__':
	main()