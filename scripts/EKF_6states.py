"""
This python module contains 6-states EKF's pridict, update and measurement function
output is Euler angle.
"""

import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import operator
import socket
import os
import numpy as np

class EKF_6states(object):
	"""docstring for EKF_6states"""
	def __init__(self, wx, wy, wz, ax, ay, az, mx, my, mz):
		super(EKF_6states, self).__init__()
		# save IMU variable
		self._wx = wx
		self._wy = wy
		self._wx = wz
		self._ax = ax
		self._ay = ay
		self._az = az
		self._mx = mx
		self._my = my
		self._mz = mz

	def Predict(self):
		pass

	def Update(self):
		pass

	def Measurement(self):
		pass