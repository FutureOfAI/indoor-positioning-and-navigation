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
import EKF_6states as EKF6

# Initialize EKF 6-states parameters
ekf6 = EKF6.EKF_6states(0.01)

d2r = np.pi/180
r2d = 180/np.pi

Roll = 90*d2r
Pitch = 60*d2r
Yaw = 30*d2r

R1 = ekf6.euler2rotMat(Roll, Pitch, Yaw)
Q1 = ekf6.rotMat2quatern(R1)
euler = ekf6.quatern2euler(Q1)*r2d

print (Q1)
print (euler)
