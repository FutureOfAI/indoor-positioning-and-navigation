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
from pyquaternion import Quaternion

class EKF_6states(object):
    """docstring for EKF_6states"""
    def __init__(self, wxp, wyp, wzp, wx, wy, wz, ax, ay, az, mx, my, mz, bgx, bgy, bgz, timestemp):
        super(EKF_6states, self).__init__()
        # save IMU variable
        self._wxp = wxp
        self._wyp = wyp
        self._wzp = wzp
        self._wx = wx
        self._wy = wy
        self._wz = wz
        self._ax = ax
        self._ay = ay
        self._az = az
        self._mx = mx
        self._my = my
        self._mz = mz
        self._bgx_h = bgx
        self._bgy_h = bgy
        self._bgz_h = bgz
        self._dt = timestemp

    def initial_states(self):
        pass

    def Predict(self):
        pass

    def Update(self):
        pass

    def Measurement(self):
        pass

    # get direction cosine matrix from gyroscope meter
    def DCM_calculate(self):
        self._wx = self._wx - self._bgx_h
        self._wy = self._wy - self._bgy_h
        self._wz = self._wz - self._bgz_h

        self._d1 = self._wx * self._dt / 2
        self._d2 = self._wy * self._dt / 2
        self._d3 = self._wz * self._dt / 2

        self._d1p = self._wxp * self._dt / 2
        self._d2p = self._wyp * self._dt / 2
        self._d3p = self._wzp * self._dt / 2

        self._d0_s = np.square(self._d1) + np.square(self._d2) + np.square(self._d3)

        self._q1 = 1 - np.square(self._d0_s)
        self._q2 = self._d1 - (self._d0_s*self._d1 + self._d3p*self._d2 + self._d2p*self._d3)/6
        self._q3 = self._d2 - (self._d0_s*self._d2 + self._d1p*self._d3 + self._d3p*self._d1)/6
        self._q4 = self._d3 - (self._d0_s*self._d3 + self._d2p*self._d1 + self._d1p*self._d2)/6

        self._delta_Q = Quaternion(self._q1, -self._q2, self._q3, self._q4)

