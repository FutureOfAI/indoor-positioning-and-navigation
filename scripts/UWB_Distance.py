"""
This python module contains DWM1000 measure distance function
output is Distance
"""
import DW1000
import monotonic
import DW1000Constants as C
import RPi.GPIO as GPIO

class UWB_Diatance(object):
	"""docstring for UWB"""
	def __init__(self, rst, iqr, css):
		super(UWB, self).__init__()
		self._rst = rst
		self._iqr = iqr
		self._css = css
		self.timePollAckSentTS = 0
		self.timePollAckReceivedTS = 0
		self.timePollReceivedTS = 0
		self.timeRangeReceivedTS = 0
		self.timePollSentTS = 0
		self.timeRangeSentTS = 0
		self.timeComputedRangeTS = 0
		self.REPLY_DELAY_TIME_US = 7000

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self._rst,GPIO.IN)
		DW1000.begin(self._iqr)
		DW1000.setup(self._css)
		DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY)
		DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
		# clear transmit status
		DW1000.registerCallback("handleSent", self.handleSent)
		# clear received status
		DW1000.registerCallback("handleReceived", self.handleReceived)
		# start reception
		self.receiver()
		# tag last activity
		self.noteActivity()

	def millis(self):
		"""
		This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
		is used to avoid having the chip stuck in an undesirable state.
		"""
		return int(round(monotonic.monotonic() * C.MILLISECONDS))

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

	def noteActivity(self):
		"""
		This function records the time of the last activity so we can know if the device is inactive or not.
		"""
		global lastActivity
		lastActivity = self.millis()

	def Anchor_resetInactive(self):
		"""
		This function restarts the default polling operation when the device is deemed inactive.
		"""
		global expectedMsgId
		DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY)
		DW1000.registerCallback("handleSent", self.handleSent)
		DW1000.registerCallback("handleReceived", self.handleReceived)
		DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
		expectedMsgId = C.POLL
		self.receiver()
		self.noteActivity()

	def receiver(self):
		"""
		This function configures the chip to prepare for a message reception.
		"""
		DW1000.newReceive()
		DW1000.receivePermanently()
		DW1000.startReceive()

	def transmitPollAck(self):
		"""
		This function sends the polling acknowledge message which is used to confirm the reception of the polling message.
		"""
		global data
		DW1000.newTransmit()
		data[0] = C.POLL_ACK
		DW1000.setDelay(self.REPLY_DELAY_TIME_US, C.MICROSECONDS)
		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def transmitRangeAcknowledge(self):
		"""
		This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
		"""
		global data
		DW1000.newTransmit()
		data[0] = C.RANGE_REPORT

		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def transmitRangeFailed(self):
		"""
		This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
		"""
		global data
		DW1000.newTransmit()
		data[0] = C.RANGE_FAILED
		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def computeRangeAsymmetric():
		"""
		This is the function which calculates the timestamp used to determine the range between the devices.
		"""
		global timeComputedRangeTS
		round1 = DW1000.wrapTimestamp(self.timePollAckReceivedTS - self.timePollSentTS)
		reply1 = DW1000.wrapTimestamp(self.timePollAckSentTS - self.timePollReceivedTS)
		round2 = DW1000.wrapTimestamp(self.timeRangeReceivedTS - self.timePollAckSentTS)
		reply2 = DW1000.wrapTimestamp(self.timeRangeSentTS - self.timePollAckReceivedTS)
		timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)

	def loop():
		global uwb_array,gyro_array,gyro_count,sentAck,n_ekf_start,start,receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId,expectedMsgID, timeRangeSentTS,Same_tag_flag,DistanceFinish_Flag,EKF_start,EKF_message,EKF_New,EKF_Update

		if sentAck == False and receivedAck == False:
			if ((millis() - lastActivity) > C.RESET_PERIOD):
				Anchor_resetInactive()
			return
		if sentAck:
			#print("1")
			sentAck = False
			msgId = data[0]
			if Same_tag_flag == data[16]:
				if msgId == C.POLL_ACK:
					timePollAckSentTS = DW1000.getTransmitTimestamp()
					noteActivity()

		if receivedAck:
			receivedAck = False
			data = DW1000.getData(LEN_DATA)
			msgId = data[0]
			if msgId == C.POLL:
				#print("2")
				DistanceFinish_Flag =1
				Same_tag_flag = data[16]
				protocolFailed = False
				timePollReceivedTS = DW1000.getReceiveTimestamp()
				expectedMsgId = C.RANGE
				transmitPollAck()
				noteActivity()
			elif msgId == C.RANGE :
				#print("3")
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
							print("Tag: %.2d"%(data[16]))
							print("Distance1: %.2f m" %(distance))
							#psierrrint("[%s]"%(time.ctime(time.time())))
							t = time.time()
							#print (int(round(t * 1000)))
							if distance <12:
								tag[0]=distance

						if data[16]==25:
							print("Tag: %.2d"%(data[16]))
							print("Distance2: %.2f m" %(distance))
							#print("[%s]"%(time.ctime(time.time())))
							if distance <12:
								tag[1]=distance

						if data[16]==26:
							print("Tag: %.2d"%(data[16]))
							print("Distance3: %.2f m" %(distance))
							t = time.time()
							#print (int(round(t * 1000)))
							if distance <12:
								tag[2]=distance

						if data[16]==27:
							print("Tag: %.2d"%(data[16]))
							print("Distance4: %.2f m" %(distance))
							if distance <12:
								tag[3]=distance
						if data[16]==24:
							print("Tag: %.2d"%(data[16]))
							print("Distance4: %.2f m" %(distance))
							if distance <12:
								tag[4]=distance
						if data[16]==29:
							print("Tag: %.2d"%(data[16]))
							print("Distance4: %.2f m" %(distance))
							if distance <12:
								tag[5]=distance

					else:
						transmitRangeFailed()

					noteActivity()