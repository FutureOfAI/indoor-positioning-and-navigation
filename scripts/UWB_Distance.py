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
	def __init__(self, arg):
		super(UWB, self).__init__()
		self.arg = arg
