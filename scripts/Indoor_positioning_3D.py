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
import UWB_Distance as UWB
from Queue import Queue
import psutil

# initial database matrix
IMU_Database = np.zeros([4000,9])
UWB_Database = np.zeros([4000,6])
IMU_Database_cnt = 0
UWB_Database_cnt = 0
IMU_Database_flag = 0
UWB_Database_flag = 0

# IMU settings
SETTINGS_FILE = "RTIMULib"
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
if (not imu.IMUInit()):
	print ("IMU Initialize Failed.")
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
poll_interval = imu.IMUGetPollInterval()

# DWM Initialize
DistanceFinish_Flag=0
protocolFailed = False
# Hardware settings
PIN_RST = 17
PIN_IRQ = 19
PIN_SS = 27
sentAck = False
receivedAck = False
lastActivity = 0
Same_tag_flag = 0
# diatance calculate params
timePollAckSentTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timePollAckReceivedTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
# data length of uwb
LEN_DATA =25
uwb_data = [0] * LEN_DATA
# DWM1000 Message types
DWM1000_POLL = 0
DWM1000_POLL_ACK = 1
DWM1000_RANGE = 2
DWM1000_RANGE_REPORT = 3
DWM1000_BLINK = 4
DWM1000_RANGE_FAILED = 255
DWM1000_RESET_PERIOD = 1000
DWM1000_TIME_OVERFLOW = 0x10000000000
# Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
DWM1000_DISTANCE_OF_RADIO = 0.0046917639786159
# set DWM1000 mutex for muti-anchors distance measure
UWB_Mutex = threading.Lock()


# Matlab algorithm test data 
grox_test = np.array([0,0.002741556236919,0.002741555560467])
groy_test = np.array([0,0.005483112473838,0.005483111120934])
groz_test = np.array([0,0.008224668710756,0.008224666681400])

accx_test = np.array([0,-0.001074778557705,-0.001612233961899])
accy_test = np.array([0,0.0005371683061720239,0.0008056197943218936])
accz_test = np.array([-9.8,-9.8,-9.8])

magx_test = np.array([2.877312458709545,2.868357795017813,2.863880397500370])
magy_test = np.array([35.997397931215560,35.999386810200676,36.000380469412846])
magz_test = np.array([27.659836572125170,27.658178118764624,27.657348736390680])
# IMU initial params
acc = np.zeros(3)
grop = np.zeros(3)
gro = np.zeros(3)
mag = np.zeros(3)

# gyro err and bias
gyro_err_flag = 1
gyro_bias_flag = 1

# EKF Initial params
r2d = 180/np.pi
d2r = np.pi/180
# previous gyro data
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
# initial gyro bias predict data
bgx_h = 0
bgy_h = 0
bgz_h = 0
# initial quaternion
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
# initial state matrix
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
# initial P matrix
s6_P00_z = np.zeros([6,6])
s6_P00_z[0,0] = np.square(phierr*d2r)
s6_P00_z[1,1] = np.square(thetaerr*d2r)
s6_P00_z[2,2] = np.square(psierr*d2r)
s6_P00_z[3,3] = np.square(bgx0)
s6_P00_z[4,4] = np.square(bgy0)
s6_P00_z[5,5] = np.square(bgz0)
# initial angle and rate random walk
sig_factor = 1
sig_x_arw = sig_factor*gyro_err_flag*0.02
sig_y_arw = sig_factor*gyro_err_flag*0.02
sig_z_arw = sig_factor*gyro_err_flag*0.02
sig_x_rrw = sig_factor*gyro_err_flag*0.02/3600
sig_y_rrw = sig_factor*gyro_err_flag*0.02/3600
sig_z_rrw = sig_factor*gyro_err_flag*0.02/3600
# initial Q matrix
s6_Q_z = np.zeros([6,6])
Q_factor = 0.01
s6_Q_z[0,0] = Q_factor*np.square(sig_x_arw)
s6_Q_z[1,1] = Q_factor*np.square(sig_y_arw)
s6_Q_z[2,2] = Q_factor*np.square(sig_z_arw)
s6_Q_z[3,3] = Q_factor*np.square(sig_x_rrw)
s6_Q_z[4,4] = Q_factor*np.square(sig_y_rrw)
s6_Q_z[5,5] = Q_factor*np.square(sig_z_rrw)
# H matrix
s6_H = np.zeros([3,6])
s6_H[0,0] = 1
s6_H[1,1] = 1
s6_H[2,2] = 1
# R matrix
s6_R = np.zeros([3,3])
R_factor = 0.1
s6_R[0,0] = R_factor*np.square(0.5*d2r)
s6_R[1,1] = R_factor*np.square(0.5*d2r)
s6_R[2,2] = R_factor*np.square(2.5*d2r)

def handleSent(self):
	"""
	This is a callback called from the module's interrupt handler when a transmission was successful.
	It sets the sentAck variable as True so the loop can continue.
	"""
	global sentAck
	sentAck = True

def handleReceived(self):
	"""
	This is a callback called from the module's interrupt handler when a reception was successful.
	It sets the received receivedAck as True so the loop can continue.
	"""
	global receivedAck
	receivedAck = True

# Initialize EKF 6-states parameters 0.01s
ekf6 = EKF6.EKF_6states(0.01)
# Initialize UWB parameters
uwb = UWB.UWB_Distance(PIN_RST, PIN_IRQ, PIN_SS, handleSent, handleReceived)
# defind reveiver
uwb.receiver()
# tag last activity
lastActivity = uwb.noteActivity()

# IMU Thread
class Get_IMU_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global acc, gro, mag, IMU_Database_cnt
		while True:
			if imu.IMURead():
				# generate IMU data
				if  IMU_Database_cnt<4000:
					IMU_Database[IMU_Database_cnt,:] = np.array([acc[0], acc[1], acc[2], gro[0], gro[1], gro[2], mag[0], mag[1], mag[2]])
				IMU_Database_cnt = IMU_Database_cnt + 1
				imu_data = imu.getIMUData()
				acc = imu_data["accel"]
				# previous gyro data
				grop = gro
				gro = imu_data["gyro"]
				mag = imu_data["compass"]
			time.sleep(0.01)

# DWM Thread
class Get_UWB_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global ( sentAck, receivedAck, lastActivity, timePollAckSentTS, uwb_data, DistanceFinish_Flag, Same_tag_flag, protocolFailed, 
			timePollReceivedTS, timeRangeReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeSentTS, timeComputedRangeTS )

		while True:
			if sentAck == False and receivedAck == False:
				if (uwb.millis() - lastActivity) > DWM1000_RESET_PERIOD:
					# reset MsgId
					lastActivity = uwb.Anchor_resetInactive()
				return

			if sentAck:
				# clear sentAck
				sentAck = False
				msgId = uwb_data[0]
				if msgId == DWM1000_POLL_ACK:
					timePollAckSentTS = uwb.get_Trans_timestamp()
					lastActivity = uwb.noteActivity()

			if receivedAck:
				receivedAck = False
				uwb_data = uwb.read_DWM1000_data(LEN_DATA)
				msgId = uwb_data[0]
				if msgId == DWM1000_POLL:
					DistanceFinish_Flag = 1
					Same_tag_flag = data[16]
					protocolFailed = False
					timePollReceivedTS = uwb.get_Rcv_timestamp()
					uwb.transmitPollAck(uwb_data, LEN_DATA)
					lastActivity = uwb.noteActivity()
				elif msgId == DWM1000_RANGE:
					if DistanceFinish_Flag == 1 and Same_tag_flag == data[16]:
						DistanceFinish_Flag = 0
						timeRangeReceivedTS = uwb.get_Rcv_timestamp()
						if protocolFailed == False:
							timePollSentTS = uwb.get_timestamp(uwb_data, 1)
							timePollAckReceivedTS = uwb.get_timestamp(uwb_data, 6)
							timeRangeSentTS = uwb.get_timestamp(uwb_data, 11)
							timeComputedRangeTS = uwb.computeRangeAsymmetric(timePollAckSentTS, timePollReceivedTS, timeRangeReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeSentTS)
							uwb.transmitRangeAcknowledge(uwb_data, LEN_DATA)
							distance = (timeComputedRangeTS % DWM1000_TIME_OVERFLOW) * DWM1000_DISTANCE_OF_RADIO
							if uwb_data[16] == 23:
								print("Tag: %.2d"%(data[16]))
								print("Distance1: %.2f m" %(distance))
							if data[16]==24:
								print("Tag: %.2d"%(data[16]))
								print("Distance4: %.2f m" %(distance))
							if data[16]==25:
								print("Tag: %.2d"%(data[16]))
								print("Distance2: %.2f m" %(distance))
							if data[16]==26:
								print("Tag: %.2d"%(data[16]))
								print("Distance3: %.2f m" %(distance))
							if data[16]==27:
								print("Tag: %.2d"%(data[16]))
								print("Distance4: %.2f m" %(distance))
							if data[16]==29:
								print("Tag: %.2d"%(data[16]))
								print("Distance4: %.2f m" %(distance))
						else:
							uwb.transmitRangeFailed(uwb_data, LEN_DATA)

						lastActivity = uwb.noteActivity()
			time.sleep(0.01)

# 6-states EKF thread
class EKF_Cal_Euler(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global w_EB_B_xm, w_EB_B_ym, w_EB_B_zm, bgx_h, bgy_h, bgz_h, QE_B_m, s6_P00_z, dtheda_xh, dtheda_yh, dtheda_zh
		while True:
			EKF_start_time = time.time()
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
			EKF_end_time = time.time()
			dt = EKF_start_time-EKF_end_time
			print (Angle*r2d, dt)
			#print (psutil.cpu_percent())
			time.sleep(0.01)

# Save data Thread
class Save_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global IMU_Database_cnt,IMU_Database_flag
		while True:
			if IMU_Database_cnt>4000:
				if IMU_Database_flag == 0:
					np.savetxt('output.csv', IMU_Database, delimiter=',')
					IMU_Database_flag = 1
					print ("IMU Dabase Full!")
			else:
				#print (IMU_Database_cnt)
				pass
			time.sleep(1)
		

# main Thread
def main():
	queue = Queue()
	imu_queue = Get_IMU_Data('IMU.', queue)
	uwb_queue = Get_UWB_Data('UWB.', queue)
	euler_queue = EKF_Cal_Euler('Euler.',queue)
	data_queue = Save_Data('Save Data', queue)
	imu_queue.start()
	uwb_queue.start()
	euler_queue.start()
	data_queue.start()
	imu_queue.join()
	uwb_queue.join()
	euler_queue.join()
	data_queue.join()
	print ('All threads terminate!')


if __name__ == '__main__':
	main()