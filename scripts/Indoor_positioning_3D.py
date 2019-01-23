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

def Get_IMU_Data():
	if imu.IMURead():
		data = imu.getIMUData()
		acc = data["accel"]
		gro = data["gyro"]
		mag = data["compass"]
	print ("task IMU")
	time.sleep(0.01)

IMU_thread = threading.Thread(target = Get_IMU_Data)
IMU_thread.start()

while True:
	print ("loop")
	time.sleep(0.01)