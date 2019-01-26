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
	def __init__(self, rst, iqr, css, handleSent, handleReceived):
		super(UWB, self).__init__()
		self._rst = rst
		self._iqr = iqr
		self._css = css
		self._handleSent = handleSent
		self._handleReceived = handleReceived
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
		# self.receiver()
		# tag last activity
		# self.noteActivity()

	def millis(self):
		"""
		This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
		is used to avoid having the chip stuck in an undesirable state.
		"""
		return int(round(monotonic.monotonic() * C.MILLISECONDS))

	def noteActivity(self):
		"""
		This function records the time of the last activity so we can know if the device is inactive or not.
		"""
		lastActivity = self.millis()
		return lastActivity

	def Anchor_resetInactive(self):
		"""
		This function restarts the default polling operation when the device is deemed inactive.
		"""
		DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9B", C.MODE_LONGDATA_RANGE_ACCURACY)
		DW1000.registerCallback("handleSent", self.handleSent)
		DW1000.registerCallback("handleReceived", self.handleReceived)
		DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)
		self.receiver()
		lastActivity = self.noteActivity()
		return lastActivity

	def receiver(self):
		"""
		This function configures the chip to prepare for a message reception.
		"""
		DW1000.newReceive()
		DW1000.receivePermanently()
		DW1000.startReceive()

	def transmitPollAck(self, data, LEN_DATA):
		"""
		This function sends the polling acknowledge message which is used to confirm the reception of the polling message.
		"""
		DW1000.newTransmit()
		data[0] = C.POLL_ACK
		DW1000.setDelay(self.REPLY_DELAY_TIME_US, C.MICROSECONDS)
		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def transmitRangeAcknowledge(self, data, LEN_DATA):
		"""
		This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
		"""
		DW1000.newTransmit()
		data[0] = C.RANGE_REPORT
		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def transmitRangeFailed(self, data, LEN_DATA):
		"""
		This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
		"""
		DW1000.newTransmit()
		data[0] = C.RANGE_FAILED
		DW1000.setData(data, LEN_DATA)
		DW1000.startTransmit()

	def computeRangeAsymmetric(self, timePollAckSentTS, timePollReceivedTS, timeRangeReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeSentTS):
		"""
		This is the function which calculates the timestamp used to determine the range between the devices.
		"""
		round1 = DW1000.wrapTimestamp(timePollAckReceivedTS - timePollSentTS)
		reply1 = DW1000.wrapTimestamp(timePollAckSentTS - timePollReceivedTS)
		round2 = DW1000.wrapTimestamp(timeRangeReceivedTS - timePollAckSentTS)
		reply2 = DW1000.wrapTimestamp(timeRangeSentTS - timePollAckReceivedTS)
		timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)
		return timeComputedRangeTS

	def read_DWM1000_data(self, LEN_DATA):
		return DW1000.getData(LEN_DATA)

	def get_Trans_timestamp(self):
		return DW1000.getTransmitTimestamp()

	def get_Rcv_timestamp(self):
		return DW1000.getReceiveTimestamp()

	def get_timestamp(self, data, pos):
		return DW1000.getTimeStamp(data, pos)