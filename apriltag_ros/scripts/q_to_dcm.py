#!/usr/bin/env python

import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mavros_msgs.msg import OverrideRCIn, VFR_HUD
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8

def q_to_euler(q):
    e1 = q[0]
    e2 = q[1]
    e3 = q[2]
    eta = q[3]
    rot = np.array([[1 - 2 * (e2**2 + e3**2),
                    2 * (e1 * e2 - e3 * eta),
                    2 * (e1 * e3 + e2 * eta)],
                    [2 * (e1 * e2 + e3 * eta),
                    1 - 2 * (e1**2 + e3**2),
                    2 * (e2 * e3 - e1 * eta)],
                    [2 * (e1 * e3 - e2 * eta),
                    2 * (e2 * e3 + e1 * eta),
                    1 - 2 * (e1**2 + e2**2)]])
    # Roll
    roll = np.arctan2(rot[2, 1], rot[2, 2]) / np.pi * 180.0
    # Pitch
    den   = np.sqrt(rot[2, 1]**2 + rot[2, 2]**2)
    pitch = np.arctan(-rot[2, 0]/ max(0.0001, den)) / np.pi * 180.0
    # Yaw
    yaw = np.arctan2(rot[1, 0], rot[0, 0]) / np.pi * 180.0

    euler = np.array([roll, pitch ,yaw])

    return euler


def q_to_dcm(q):
    e1 = q[0]
    e2 = q[1]
    e3 = q[2]
    eta = q[3]

    R = np.array([[1-2*(e2**2+e3**2),
        2*(e1*e2-e3*eta),
        2*(e1*e3+e2*eta)],
        [2*(e1*e2+e3*eta),
        1-2*(e1**2+e3**2),
        2*(e2*e3-e1*eta)],
        [2*(e1*e3-e2*eta),
        2*(e2*e3+e1*eta),
        1-2*(e1**2+e2**2)]])
    
    return R


q = np.zeros(4)

q[0] = -0.5768
q[1] = -0.0148
q[2] = -0.020
q[3] = -0.8162

euler = q_to_euler(q)
dcm = q_to_dcm(q)

print(euler)
print(dcm)   # dcm: c2w

Rc_w = dcm

Rci_b = np.zeros(shape=(3,3))
Rci_b = np.array([[0.0, 0.0, 1.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]])


Rb_w = np.zeros(shape=(3,3))
Rb_w = np.array([[0.0, 1.0, 0.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])




# Rb_w = Rci_w*Rc_ci*Rb_c

r1 = np.dot(np.transpose(Rc_w), Rb_w)
Rci_c = np.dot(r1, Rci_b)
print('-------')
print(Rci_c)

r2 = np.dot(Rci_c, np.transpose(Rci_b))
print(r2)

Rb_c = np.dot(np.transpose(Rc_w),Rb_w)
print(Rb_c)


q = np.zeros(4)

q[0] = 0.0
q[1] = 0.0
q[2] = -0.8930
q[3] = 0.4496

dcm = q_to_dcm(q)
euler =  q_to_euler(q)
print(dcm)
print(euler)




