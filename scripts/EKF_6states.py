"""
This python module contains 6-states EKF's pridict, update and measurement function
output is Euler angle.
"""

# import sys, getopt

# sys.path.append('.')
# import os.path
# import time
# import math
# import operator
# import socket
# import os
import numpy as np
from pyquaternion import Quaternion
from numpy import linalg as LA

class EKF_6states(object):
    """docstring for EKF_6states"""
    def __init__(self, timestemp):
        super(EKF_6states, self).__init__()
        self._d2r = np.pi/180
        self._r2d = 180/np.pi
        self._Mag = 45.488
        self._Angle_I = 1*37.45*self._d2r;
        self._Angle_D = 1*4.57*self._d2r;
        self._dt = timestemp

    def initial_states(self):
        pass

    def Predict(self):
        pass

    def Update(self):
        pass

    def Measurement(self):
        pass

    # get direction cosine matrix from gyroscopemeter
    def DCM_calculate(self, wxp, wyp, wzp, wx, wy, wz, bgx_h, bgy_h, bgz_h, QE_B_m):
        wx = wx - bgx_h
        wy = wy - bgy_h
        wz = wz - bgz_h

        d1 = wx * self._dt / 2
        d2 = wy * self._dt / 2
        d3 = wz * self._dt / 2

        d1p = wxp * self._dt / 2
        d2p = wyp * self._dt / 2
        d3p = wzp * self._dt / 2

        d0_s = np.square(d1) + np.square(d2) + np.square(d3)

        q1 = 1 - d0_s/2
        q2 = d1 - (d0_s*d1 + d3p*d2 + d2p*d3)/6
        q3 = d2 - (d0_s*d2 + d1p*d3 + d3p*d1)/6
        q4 = d3 - (d0_s*d3 + d2p*d1 + d1p*d2)/6

        delta_Q = Quaternion(q1, -q2, -q3, -q4)
        QE_B_m = delta_Q.normalised * QE_B_m.normalised
        DC_E_B_m = QE_B_m.rotation_matrix

        return DC_E_B_m

    # Get Rotation matrix from euler angle in rad
    def euler2rotMat(self, phi, theta, psi):
        R = np.zeros([3,3])

        R[0,0] = np.cos(psi)*np.cos(theta)
        R[0,1] = -np.sin(psi)*np.cos(phi) + np.cos(psi)*np.sin(theta)*np.sin(phi)
        R[0,2] = np.sin(psi)*np.sin(phi) + np.cos(psi)*np.sin(theta)*np.cos(phi)

        R[1,0] = np.sin(psi)*np.cos(theta)
        R[1,1] = np.cos(psi)*np.cos(phi) + np.sin(psi)*np.sin(theta)*np.sin(psi)
        R[1,2] = -np.cos(psi)*np.sin(phi) + np.sin(psi)*np.sin(theta)*np.cos(phi)

        R[2,0] = -np.sin(theta)
        R[2,1] = np.cos(theta)*np.sin(phi)
        R[2,2] = np.cos(theta)*np.cos(phi)

        return R

    def rotMat2quatern(self, R):
        K = zeros([4,4])

        K[0,0] = (1/3)*( R[0,0] - R[1,1] - R[2,2] )
        K[0,1] = (1/3)*( R[1,0] + R[0,1] )
        K[0,2] = (1/3)*( R[2,0] + R[0,2] )
        K[0,3] = (1/3)*( R[1,2] - R[2,1] )

        K[1,0] = (1/3)*( R[1,0] + R[0,1] )
        K[1,1] = (1/3)*( R[1,1] - R[0,0] - R[2,2] )
        K[1,2] = (1/3)*( R[2,1] + R[1,2] )
        K[1,3] = (1/3)*( R[2,0] - R[0,2] )

        K[2,0] = (1/3)*( R[2,0] + R[0,2] )
        K[2,1] = (1/3)*( R[2,1] + R[1,2] )
        K[2,2] = (1/3)*( R[2,2] - R[0,0] - R[1,1] )
        K[2,3] = (1/3)*( R[1,2] - R[2,1] )

        K[3,0] = (1/3)*( R[1,2] - R[2,1] )
        K[3,1] = (1/3)*( R[2,0] - R[0,2] )
        K[4,2] = (1/3)*( R[0,1] - R[1,0] )
        K[3,3] = (1/3)*( R[0,0] + R[1,1] + R[2,2] )

        [V,D] = LA.eig(K)
        q = Quaternion(V)
        return q

    def quatern2euler(self, q):
        R = q.rotation_matrix

        phi = np.arctan2(R[2,1], R[2,2])
        theta = -np.arctan( R[2,0]/np.sqrt( 1 - np.square(R[2,0]) ) )
        psi = np.arctan2(R[1,0], R[0,0])

        euler = np.array([phi, theta, psi])
        return euler

    def TRIAD(self, ax, ay, az, mx, my, mz):
        acc_g = np.array([0, 0, -9.8])
        mag_E = np.array([self._Mag*np.cos(self._Angle_I)*np.sin(self._Angle_D), self._Mag*np.cos(self._Angle_I)*np.cos(self._Angle_D), -self._Mag*np.sin(self._Angle_I)])

        a_B = np.array([ax, ay, az])
        q_B = a_B/LA.norm(a_B)
        m_B = np.array([mx, my, mz])
        m_B_u = m_B/LA.norm(m_B)
        r_B_n = np.cross(q_B, m_B_u)
        r_B = r_B_n/LA.norm(r_B_n)
        s_B = np.cross(q_B, r_B)
        M_B = np.array([s_B, r_B, q_B])
        norm_M_B = M_B/LA.norm(M_B)

        q_E = acc_g/LA.norm(acc_g)
        mag_E_u = mag_E/LA.norm(mag_E)
        r_E_n = np.cross(q_E, mag_E_u)
        r_E = r_E_n/LA.norm(r_E_n)
        s_E = np.cross(q_E, r_E)
        M_E = np.array([s_E, r_E, q_E])
        norm_M_E = M_E/LA.norm(M_E)

        C_E_B_e = norm_M_B.reshape([3,1]).dot(norm_M_E.reshape([1,3]))
        return C_E_B_e
