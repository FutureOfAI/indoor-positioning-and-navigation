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
import psutil

grox_test = np.array([0,0.002741556236919,0.002741555560467])
groy_test = np.array([0,0.005483112473838,0.005483111120934])
groz_test = np.array([0,0.008224668710756,0.008224666681400])

accx_test = np.array([0,-0.001074778557705,-0.001612233961899])
accy_test = np.array([0,0.0005371683061720239,0.0008056197943218936])
accz_test = np.array([-9.8,-9.8,-9.8])

magx_test = np.array([2.877312458709545,2.868357795017813,2.863880397500370])
magy_test = np.array([35.997397931215560,35.999386810200676,36.000380469412846])
magz_test = np.array([27.659836572125170,27.658178118764624,27.657348736390680])

acc = np.zeros(3)
gro = np.zeros(3)
mag = np.zeros(3)

# Initialize EKF 6-states parameters 0.01s
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
poll_interval = imu.IMUGetPollInterval()

gyro_err_flag = 0
gyro_bias_flag = 0

# EKF Initial params
r2d = 180/np.pi
d2r = np.pi/180

w_EB_B_xm = 0
w_EB_B_ym = 0
w_EB_B_zm = 0

# Euler error in deg
phierr = 1*0.5
thetaerr = -1*0.5
psierr = 10*0.5

dtheda_xh = phierr*d2r
dtheda_yh = thetaerr*d2r
dtheda_zh = psierr*d2r

bgx_h = 0
bgy_h = 0
bgz_h = 0

dq11 = -dtheda_xh/2
dq21 = -dtheda_yh/2
dq31 = -dtheda_zh/2
q2 = -dq11
q3 = -dq21
q4 = -dq31
q1 = np.sqrt(1-np.square(q2)-np.square(q3)-np.square(q4))
dQerr = Quaternion(q1, q2, q3, q4)
Q_E_B = Quaternion(1, 0, 0, 0)
QE_B_m = dQerr.normalised * Q_E_B.normalised

bgx0=gyro_bias_flag*0.05*d2r
bgy0=gyro_bias_flag*(-0.05)*d2r
bgz0=gyro_bias_flag*0.05*d2r

s6_xz_h = np.zeros([6,1])
s6_xz_h[0,0] = phierr*d2r
s6_xz_h[1,0] = thetaerr*d2r
s6_xz_h[2,0] = psierr*d2r
s6_xz_h[3,0] = bgx0
s6_xz_h[4,0] = bgy0
s6_xz_h[5,0] = bgz0

s6_P00_z = np.zeros([6,6])
s6_P00_z[0,0] = np.square(phierr*d2r)
s6_P00_z[1,1] = np.square(thetaerr*d2r)
s6_P00_z[2,2] = np.square(psierr*d2r)
s6_P00_z[3,3] = np.square(bgx0)
s6_P00_z[4,4] = np.square(bgy0)
s6_P00_z[5,5] = np.square(bgz0)

sig_x_arw = gyro_err_flag*0.02
sig_y_arw = gyro_err_flag*0.02
sig_z_arw = gyro_err_flag*0.02
sig_x_rrw = gyro_err_flag*0.02/3600
sig_y_rrw = gyro_err_flag*0.02/3600
sig_z_rrw = gyro_err_flag*0.02/3600

s6_Q_z = np.zeros([6,6])
Q_factor = 1
s6_Q_z[0,0] = Q_factor*np.square(sig_x_arw)
s6_Q_z[1,1] = Q_factor*np.square(sig_y_arw)
s6_Q_z[2,2] = Q_factor*np.square(sig_z_arw)
s6_Q_z[3,3] = Q_factor*np.square(sig_x_rrw)
s6_Q_z[4,4] = Q_factor*np.square(sig_y_rrw)
s6_Q_z[5,5] = Q_factor*np.square(sig_z_rrw)

s6_H = np.zeros([3,6])
s6_H[0,0] = 1
s6_H[1,1] = 1
s6_H[2,2] = 1

s6_R = np.zeros([3,3])
R_factor = 0.1
s6_R[0,0] = R_factor*np.square(0.5*d2r)
s6_R[1,1] = R_factor*np.square(0.5*d2r)
s6_R[2,2] = R_factor*np.square(2.5*d2r)

# IMU Thread
class Get_IMU_Data(threading.Thread):
	def __init__(self, tasks):
		threading.Thread.__init__(self)
		self.tasks = tasks
		self.start()
	def run(self):
		global acc, gro, mag
		while True:
			if imu.IMURead():
				data = imu.getIMUData()
				acc = data["accel"]
				gro = data["gyro"]
				mag = data["compass"]
			time.sleep(poll_interval*2/1000)

# DWM Thread
class Get_UWB_Data(threading.Thread):
	def __init__(self, tasks):
		threading.Thread.__init__(self)
		self.tasks = tasks
		self.start()
	def run(self):
		while True:
			#print ("task-UWB")
			time.sleep(1)

# 6-states EKF thread
class EKF_Cal_Euler(threading.Thread):
	def __init__(self, tasks):
		threading.Thread.__init__(self)
		self.tasks = tasks
		self.start()
	def run(self):
		while True:
			global w_EB_B_xm, w_EB_B_ym, w_EB_B_zm, bgx_h, bgy_h, bgz_h, QE_B_m, s6_P00_z, dtheda_xh, dtheda_yh, dtheda_zh
			start_time = time.time()
			# predict
			s6_P00_z, QE_B_m = ekf6.Predict(w_EB_B_xm, w_EB_B_ym, w_EB_B_zm, gro[0], gro[1], gro[2], bgx_h, bgy_h, bgz_h, QE_B_m, s6_xz_h, s6_P00_z, s6_Q_z)
			# update
			s6_P00_z, s6_z_update = ekf6.Update(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], QE_B_m, s6_P00_z, s6_H, s6_R)
			# measurement
			dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm = ekf6.Measurement(dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, s6_z_update, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm)
			# calculate euler angle
			q2 = -dtheda_xh/2
			q3 = -dtheda_yh/2
			q4 = -dtheda_zh/2
			q1 = np.sqrt(1-np.square(q2)-np.square(q3)-np.square(q4))
			dQ2 = Quaternion(q1, q2, q3, q4)
			QE_B_m = dQ2.normalised * QE_B_m.normalised
			Angle = ekf6.quatern2euler(QE_B_m)
			print (Angle*r2d)
			end_time = time.time()
			#print (psutil.cpu_percent())
			#print (end_time-start_time)
			#time.sleep(0.5)

class ThreadPool:
	"""Pool of threads consuming tasks from a queue"""
	def __init__(self, num_threads):
		self.tasks = Queue(num_threads)
		while True:
			Get_IMU_Data(self.tasks)
			Get_UWB_Data(self.tasks)
			EKF_Cal_Euler(self.tasks)

	def add_task(self, func, *args, **kargs):
		"""Add a task to the queue"""
		self.tasks.put((func, args, kargs))

	def wait_completion(self):
		"""Wait for completion of all the tasks in the queue"""
		self.tasks.join()

# main Thread
def main():
	pool = ThreadPool(20)
	pool.wait_completion()
	print ('All threads terminate!')

if __name__ == '__main__':
	main()