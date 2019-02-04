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
from scipy import linalg as LA

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

    def Predict(self, wxp, wyp, wzp, wx, wy, wz, bgx_h, bgy_h, bgz_h, QE_B_m, s6_xz_h, s6_P00_z, s6_Q_z):
        s6_F_z = np.zeros([6,6])

        DC_E_B_m, QE_B_m = self.DCM_calculate(wxp, wyp, wzp, wx, wy, wz, bgx_h, bgy_h, bgz_h, QE_B_m)

        s6_F_z[0,3] = -DC_E_B_m[0,0]
        s6_F_z[0,4] = -DC_E_B_m[1,0]
        s6_F_z[0,5] = -DC_E_B_m[2,0]

        s6_F_z[1,3] = -DC_E_B_m[0,1]
        s6_F_z[1,4] = -DC_E_B_m[1,1]
        s6_F_z[1,5] = -DC_E_B_m[2,1]

        s6_F_z[2,3] = -DC_E_B_m[0,2]
        s6_F_z[2,3] = -DC_E_B_m[1,2]
        s6_F_z[2,3] = -DC_E_B_m[2,2]

        s6_phi_z = LA.expm(s6_F_z*self._dt)
        s6_xz_h = s6_phi_z.dot(s6_xz_h)
        s6_P00_z = s6_phi_z.dot(s6_P00_z).dot(s6_phi_z.T) + s6_Q_z*self._dt

        return s6_P00_z, QE_B_m

    def Update(self, ax, ay, az, mx, my, mz, QE_B_m, s6_P00_z, s6_H, s6_R):

        ax = ax*9.8
        ay = ay*9.8
        az = -az*9.8

        C_E_B_e = self.TRIAD(ax, ay, az, mx, my, mz)
        # print (C_E_B_e)
        tmp = self.rotMat2euler(C_E_B_e.T)
        # C_E_B_e = self.euler2rotMat(-tmp[1], tmp[0], tmp[2])
        # print (tmp)
        # tmp = self.AccMag2euler(ax, ay, az, mx, my)
        # C_E_B_e = self.euler2rotMat(tmp[0],tmp[1],tmp[2])
        # Q_E_B_e = self.rotMat2quatern(C_E_B_e)
        Q_E_B_e = self.euler2quatern(-tmp[1], tmp[0], tmp[2])
        # print (Q_E_B_e[0],Q_E_B_e[1],Q_E_B_e[2],Q_E_B_e[3]) @@
        Q_B_E_m = Quaternion(QE_B_m[0], -QE_B_m[1], -QE_B_m[2], -QE_B_m[3])
        dQ = Q_E_B_e.normalised * Q_B_E_m.normalised
        # print (dQ[0],dQ[1],dQ[2],dQ[3])
        d_theta = self.quatern2euler(dQ.normalised)
        # Form the measurement residuals or mu
        s6_Mu_z = d_theta
        # print (s6_Mu_z)
        # Computer the Kalman filter gain matrix K
        s6_K_z = s6_P00_z.dot(s6_H.T).dot( LA.inv(s6_H.dot(s6_P00_z).dot(s6_H.T) + s6_R) )
        # Computer the correction vectors
        s6_z_update = s6_K_z.dot(s6_Mu_z.T)
        # Perform the Kalman filter error covariance matrix P updates
        s6_P00_z = (np.identity(6) - s6_K_z.dot(s6_H)).dot(s6_P00_z)

        return s6_P00_z, s6_z_update

    def Update_v2(self, roll, pitch, yaw, QE_B_m, s6_P00_z, s6_H, s6_R):
        
        Q_E_B_e = self.euler2quatern(roll, pitch, yaw)
        Q_B_E_m = Quaternion(QE_B_m[0], -QE_B_m[1], -QE_B_m[2], -QE_B_m[3])
        # print (Q_E_B_e[0],Q_E_B_e[1],Q_E_B_e[2],Q_E_B_e[3])
        dQ = Q_E_B_e.normalised * Q_B_E_m.normalised
        # print (dQ[0],dQ[1],dQ[2],dQ[3])
        d_theta = self.quatern2euler(dQ.normalised)
        # Form the measurement residuals or mu
        s6_Mu_z = d_theta
        # print (s6_Mu_z)
        # Computer the Kalman filter gain matrix K
        s6_K_z = s6_P00_z.dot(s6_H.T).dot( LA.inv(s6_H.dot(s6_P00_z).dot(s6_H.T) + s6_R) )
        # Computer the correction vectors
        s6_z_update = s6_K_z.dot(s6_Mu_z.T)
        # Perform the Kalman filter error covariance matrix P updates
        s6_P00_z = (np.identity(6) - s6_K_z.dot(s6_H)).dot(s6_P00_z)

        return s6_P00_z, s6_z_update

    def Measurement(self, dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, s6_z_update, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm):

        dtheda_xh = s6_z_update[0]
        dtheda_yh = s6_z_update[1]
        dtheda_zh = s6_z_update[2]

        bgx_h = bgx_h + s6_z_update[3]
        bgy_h = bgy_h + s6_z_update[4]
        bgz_h = bgz_h + s6_z_update[5]

        w_EB_B_xm = w_EB_B_xm - bgx_h
        w_EB_B_ym = w_EB_B_ym - bgy_h
        w_EB_B_zm = w_EB_B_zm - bgz_h

        return dtheda_xh, dtheda_yh, dtheda_zh, bgx_h, bgy_h, bgz_h, w_EB_B_xm, w_EB_B_ym, w_EB_B_zm

    # get direction cosine matrix from gyroscopemeter
    def DCM_calculate(self, wxp, wyp, wzp, wx, wy, wz, bgx_h, bgy_h, bgz_h, QE_B_m):

        # wx = (1+0.254)*wx
        # wxp = (1+0.254)*wxp
        # wy = (1+0.246)*wy
        # wyp = (1+0.246)*wyp
        # wz = (1+0.24)*wz
        # wzp = (1+0.24)*wzp

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
        DC_E_B_m = self.quatern2rotMat(QE_B_m)
        # print (QE_B_m[0],QE_B_m[1],QE_B_m[2],QE_B_m[3])
        return DC_E_B_m, QE_B_m

    # Get Rotation matrix from euler angle in rad
    def euler2rotMat(self, phi, theta, psi):
        R = np.zeros([3,3])

        R[0,0] = np.cos(psi)*np.cos(theta)
        R[0,1] = -np.sin(psi)*np.cos(phi) + np.cos(psi)*np.sin(theta)*np.sin(phi)
        R[0,2] = np.sin(psi)*np.sin(phi) + np.cos(psi)*np.sin(theta)*np.cos(phi)

        R[1,0] = np.sin(psi)*np.cos(theta)
        R[1,1] = np.cos(psi)*np.cos(phi) + np.sin(psi)*np.sin(theta)*np.sin(phi)
        R[1,2] = -np.cos(psi)*np.sin(phi) + np.sin(psi)*np.sin(theta)*np.cos(phi)

        R[2,0] = -np.sin(theta)
        R[2,1] = np.cos(theta)*np.sin(phi)
        R[2,2] = np.cos(theta)*np.cos(phi)

        return R

    def rotMat2quatern(self, R):
        K = np.zeros([4,4])

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
        K[2,3] = (1/3)*( R[0,1] - R[1,0] )

        K[3,0] = (1/3)*( R[1,2] - R[2,1] )
        K[3,1] = (1/3)*( R[2,0] - R[0,2] )
        K[3,2] = (1/3)*( R[0,1] - R[1,0] )
        K[3,3] = (1/3)*( R[0,0] + R[1,1] + R[2,2] )

        vals,vecs = LA.eigh(K)
        # print (vecs) @@
        q = Quaternion([vecs[3,3], vecs[0,3], vecs[1,3], vecs[2,3]])
        return q

    def rotMat2euler(self, R):
        phi = np.arctan2(R[2,1], R[2,2])
        theta = -np.arctan( R[2,0]/np.sqrt( 1 - np.square(R[2,0]) ) )
        psi = np.arctan2(R[1,0], R[0,0])

        euler = np.array([phi, theta, psi])
        return euler

    def quatern2euler(self, q):
        R = self.quatern2rotMat(q)
        euler = self.rotMat2euler(R)
        return euler

    def quatern2rotMat(self, q):
        R = np.zeros([3,3])

        R[0,0] = 2*np.square(q[0])-1+2*np.square(q[1])
        R[0,1] = 2*(q[1]*q[2]+q[0]*q[3])
        R[0,2] = 2*(q[1]*q[3]-q[0]*q[2])

        R[1,0] = 2*(q[1]*q[2]-q[0]*q[3])
        R[1,1] = 2*np.square(q[0])-1+2*np.square(q[2])
        R[1,2] = 2*(q[2]*q[3]+q[0]*q[1])

        R[2,0] = 2*(q[1]*q[3]+q[0]*q[2])
        R[2,1] = 2*(q[2]*q[3]-q[0]*q[1])
        R[2,2] = 2*np.square(q[0])-1+2*np.square(q[3])

        return R

    def TRIAD(self, ax, ay, az, mx, my, mz):
        acc_g = np.array([0, 0, -9.8])
        mag_E = np.array([self._Mag*np.cos(self._Angle_I)*np.sin(self._Angle_D), self._Mag*np.cos(self._Angle_I)*np.cos(self._Angle_D), -self._Mag*np.sin(self._Angle_I)])

        # Rot_mag = self.euler2rotMat(0,np.pi,0)
        # [mx,my,mz] = Rot_mag.dot(np.array([mx,my,mz]))

        a_B = np.array([ax, ay, az])
        # print (a_B[0],a_B[1],a_B[2])
        q_B = self.normalize(a_B)
        # print (q_B[0],q_B[1],q_B[2])
        m_B = np.array([mx, my, mz])
        m_B_u = self.normalize(m_B)
        # print (m_B_u[0],m_B_u[1],m_B_u[2])
        r_B_n = np.cross(q_B, m_B_u)
        r_B = self.normalize(r_B_n)
        # print (r_B_n)
        s_B = np.cross(q_B, r_B)
        M_B = np.array([s_B, r_B, q_B])
        # print (M_B)

        q_E = self.normalize(acc_g)
        mag_E_u = self.normalize(mag_E)
        r_E_n = np.cross(q_E, mag_E_u)
        r_E = self.normalize(r_E_n)
        s_E = np.cross(q_E, r_E)
        M_E = np.array([s_E, r_E, q_E])

        C_E_B_e = M_B.dot(M_E.T)
        return C_E_B_e

    def AccMag2euler(self,ax, ay, az, mx, my):
        mag_E = np.array([self._Mag*np.cos(self._Angle_I)*np.sin(self._Angle_D), self._Mag*np.cos(self._Angle_I)*np.cos(self._Angle_D), -self._Mag*np.sin(self._Angle_I)])
        if az != 0:
            phi = -np.arctan(ay/az)
            theta = -np.arctan(ax/np.sqrt(np.square(ay)+np.square(az)))
        else:
            phi = 0
            theta = 0
        psi = np.arctan2(my, mx) - np.arctan2(mag_E[1], mag_E[0])
        euler = np.array([phi, theta, psi])
        return euler

    def normalize(self, v):
        norm_v = np.linalg.norm(v)
        if norm_v == 0:
            return v
        return v/norm_v

    def euler2quatern(self,roll,pitch,yaw):
        cy = np.cos(yaw/2)
        sy = np.sin(yaw/2)
        cp = np.cos(pitch/2)
        sp = np.sin(pitch/2)
        cr = np.cos(roll/2)
        sr = np.sin(roll/2)

        q1 = cy * cp * cr + sy * sp * sr
        q2 = cy * cp * sr - sy * sp * cr
        q3 = sy * cp * sr + cy * sp * cr
        q4 = sy * cp * cr - cy * sp * sr

        Q = Quaternion(q1, -q2, -q3, -q4)
        return Q