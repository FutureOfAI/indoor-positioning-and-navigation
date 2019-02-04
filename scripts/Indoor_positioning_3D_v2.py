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
import DW1000
import monotonic
import DW1000Constants as C
import RPi.GPIO as GPIO 
import psutil

# initial database matrix
IMU_Database = np.zeros([4000,12])
UWB_Database = np.zeros([400,6])
UWB_Databuf = np.zeros(6)
IMU_Database_cnt = 0
UWB_Database_cnt = 0
IMU_Database_flag = 0
UWB_Database_flag = 0

# Initialize EKF 6-states parameters 0.01s
ekf6 = EKF6.EKF_6states(0.01)

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
Same_tag_flag=0
lastActivity = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA =25
data = [0] * LEN_DATA
timePollAckSentTS = 0
timePollAckReceivedTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
REPLY_DELAY_TIME_US = 7000 

PIN_RST = 17
PIN_IRQ = 19
PIN_SS = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_RST,GPIO.IN)
DW1000.begin(PIN_IRQ)
DW1000.setup(PIN_SS)
# print("DW1000 initialized")
# print("############### ANCHOR ##############")
DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY)
DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

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
pose = np.zeros(3)

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


def millis():
	"""
	This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
	is used to avoid having the chip stuck in an undesirable state.
	"""    
	return int(round(monotonic.monotonic() * C.MILLISECONDS))

def handleSent():
	"""
	This is a callback called from the module's interrupt handler when a transmission was successful. 
	It sets the sentAck variable as True so the loop can continue.
	"""            
	global sentAck
	sentAck = True

def handleReceived():
	"""
	This is a callback called from the module's interrupt handler when a reception was successful. 
	It sets the received receivedAck as True so the loop can continue.
	"""
	global receivedAck
	receivedAck = True

def noteActivity():
	"""
	This function records the time of the last activity so we can know if the device is inactive or not.
	"""        
	global lastActivity
	lastActivity = millis()

def Anchor_resetInactive():
	"""
	This function restarts the default polling operation when the device is deemed inactive.
	"""    
	global expectedMsgId
	DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY) 
	DW1000.registerCallback("handleSent", handleSent)
	DW1000.registerCallback("handleReceived", handleReceived)
	DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)       
	expectedMsgId = C.POLL
	#print("run")
	receiver()
	noteActivity()

def transmitPollAck():
	"""
	This function sends the polling acknowledge message which is used to confirm the reception of the polling message. 
	"""        
	global data
	DW1000.newTransmit()
	data[0] = C.POLL_ACK
	DW1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
	DW1000.setData(data, LEN_DATA)
	DW1000.startTransmit()

def transmitRangeAcknowledge():
	"""
	This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
	"""
	global data
	DW1000.newTransmit()
	data[0] = C.RANGE_REPORT

	DW1000.setData(data, LEN_DATA)
	DW1000.startTransmit()

def transmitRangeFailed():
	"""
	This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
	"""    
	global data
	DW1000.newTransmit()
	data[0] = C.RANGE_FAILED
	DW1000.setData(data, LEN_DATA)
	DW1000.startTransmit()

def receiver():
	"""
	This function configures the chip to prepare for a message reception.
	"""
	global data
	DW1000.newReceive()
	DW1000.receivePermanently()
	DW1000.startReceive()

def computeRangeAsymmetric():
	"""
	This is the function which calculates the timestamp used to determine the range between the devices.
	"""
	global timeComputedRangeTS
	round1 = DW1000.wrapTimestamp(timePollAckReceivedTS - timePollSentTS)
	reply1 = DW1000.wrapTimestamp(timePollAckSentTS - timePollReceivedTS)
	round2 = DW1000.wrapTimestamp(timeRangeReceivedTS - timePollAckSentTS)
	reply2 = DW1000.wrapTimestamp(timeRangeSentTS - timePollAckReceivedTS)
	timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)

def loop():
	global sentAck,receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, \
		protocolFailed, data, expectedMsgId, timeRangeSentTS, Same_tag_flag, DistanceFinish_Flag, UWB_Database_cnt

	if sentAck == False and receivedAck == False:
		if ((millis() - lastActivity) > C.RESET_PERIOD):
			Anchor_resetInactive()
			return

	if sentAck:
		sentAck = False
		msgId = data[0]
		if msgId == C.POLL_ACK:
			timePollAckSentTS = DW1000.getTransmitTimestamp()
			noteActivity()

	if receivedAck:
		receivedAck = False
		data = DW1000.getData(LEN_DATA)
		msgId = data[0]
		if msgId != expectedMsgId:
			protocolFailed = True
		if msgId == C.POLL:
			DistanceFinish_Flag =1
			Same_tag_flag = data[16]
			protocolFailed = False
			timePollReceivedTS = DW1000.getReceiveTimestamp()
			expectedMsgId = C.RANGE
			transmitPollAck()
			noteActivity()
		elif msgId == C.RANGE :
			if (DistanceFinish_Flag == 1 and Same_tag_flag == data[16]):
				DistanceFinish_Flag = 0
			timeRangeReceivedTS = DW1000.getReceiveTimestamp()
			expectedMsgId = C.POLL
			if protocolFailed == False:
				timePollSentTS = DW1000.getTimeStamp(data, 1)
				timePollAckReceivedTS = DW1000.getTimeStamp(data, 6)
				timeRangeSentTS = DW1000.getTimeStamp(data, 11)
				computeRangeAsymmetric()
				transmitRangeAcknowledge()
				distance = (timeComputedRangeTS % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
				if data[16]==23:
					UWB_Databuf[0] = distance
					# print("An23_Distance: %.2f m" %(distance))
				if data[16]==25:
					UWB_Databuf[1] = distance
					# print("An25_Distance: %.2f m" %(distance))
				if data[16]==26:
					UWB_Databuf[2] = distance
					# print("An26_Distance: %.2f m" %(distance))
				if data[16]==27:
					UWB_Databuf[3] = distance
					# print("An27_Distance: %.2f m" %(distance))
				if data[16]==24:
					UWB_Databuf[4] = distance
					# print("An24_Distance: %.2f m" %(distance))
				if data[16]==29:
					UWB_Databuf[5] = distance
					# print("An29_Distance: %.2f m" %(distance))
				if UWB_Database_cnt<400:
					UWB_Database[UWB_Database_cnt,:] = np.array([UWB_Databuf[0], UWB_Databuf[1], UWB_Databuf[2], UWB_Databuf[3], UWB_Databuf[4], UWB_Databuf[5]])
				UWB_Database_cnt = UWB_Database_cnt + 1
			else:
				transmitRangeFailed()
			noteActivity()

# IMU Thread
class Get_IMU_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global acc, gro, mag, pos, IMU_Database_cnt
		while True:
			if imu.IMURead():
				imu_data = imu.getIMUData()
				acc = imu_data["accel"]
				# previous gyro data
				grop = gro
				gro = imu_data["gyro"]
				mag = imu_data["compass"]
				pose = imu_data["fusionPose"]
				# generate IMU data
				if  IMU_Database_cnt<4000:
					IMU_Database[IMU_Database_cnt,:] = np.array([acc[0], acc[1], acc[2], gro[0], gro[1], gro[2], mag[0], mag[1], mag[2], pose[0], pose[1], pose[2]])
				IMU_Database_cnt = IMU_Database_cnt + 1
			time.sleep(0.01)

# DWM Thread
class Get_UWB_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		while True:
			loop()
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
			# s6_P00_z, s6_z_update = ekf6.Update(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], QE_B_m, s6_P00_z, s6_H, s6_R)
			s6_P00_z, s6_z_update = ekf6.Update_v2(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], pose[0], pose[1], pose[2], QE_B_m, s6_P00_z, s6_H, s6_R)
			# measurement
			dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm = ekf6.Measurement(dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, s6_z_update, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm)
			# calculate euler angle
			# q2 = -dtheda_xh/2
			# q3 = -dtheda_yh/2
			# q4 = -dtheda_zh/2
			# q1 = np.sqrt(1-np.square(q2)-np.square(q3)-np.square(q4))
			# DCM_err = ekf6.euler2rotMat(dtheda_xh, dtheda_yh, dtheda_zh)
			# dQ2 = Quaternion(q1, q2, q3, q4)
			# dQ2 = ekf6.rotMat2quatern(DCM_err)
			dQ2 = ekf6.euler2quatern(dtheda_xh, dtheda_yh, dtheda_zh)
			QE_B_m = dQ2.normalised * QE_B_m.normalised
			Angle = ekf6.quatern2euler(QE_B_m)
			EKF_end_time = time.time()
			dt = EKF_start_time-EKF_end_time
			# print w_EB_B_xm,w_EB_B_ym,w_EB_B_zm
			# print (Angle*r2d)
			# print (psutil.cpu_percent())
			time.sleep(0.01)

# Save data Thread
class Save_Data(threading.Thread):
	def __init__(self, t_name, queue):
		threading.Thread.__init__(self, name = t_name)
		self.data = queue
	def run(self):
		global IMU_Database_cnt,IMU_Database_flag, UWB_Database_cnt, UWB_Database_flag
		while True:
			if IMU_Database_cnt>4000:
				if IMU_Database_flag == 0:
					np.savetxt('output_IMU.csv', IMU_Database, delimiter=',')
					IMU_Database_flag = 1
					print ("IMU Dabase Full!")
			else:
				print("IMU Counter: %d" %(IMU_Database_cnt))

			if UWB_Database_cnt>400:
				if UWB_Database_flag == 0:
					np.savetxt('output_UWB.csv', UWB_Database, delimiter=',')
					UWB_Database_flag = 1
					print ("UWB Dabase Full!")
			else:
				print("UWB Counter: %d" %(UWB_Database_cnt))
			time.sleep(1)
		

# main Thread
def main():
	# initialize DWM settings
	DW1000.registerCallback("handleSent", handleSent)
	DW1000.registerCallback("handleReceived", handleReceived)
	receiver()
	noteActivity()

	queue = Queue()
	imu_queue = Get_IMU_Data('IMU.', queue)
	# uwb_queue = Get_UWB_Data('UWB.', queue)
	euler_queue = EKF_Cal_Euler('Euler.',queue)
	# data_queue = Save_Data('Save Data', queue)
	imu_queue.start()
	# uwb_queue.start()
	euler_queue.start()
	# data_queue.start()
	imu_queue.join()
	# uwb_queue.join()
	euler_queue.join()
	# data_queue.join()
	print ('All threads terminate!')


if __name__ == '__main__':
	main()