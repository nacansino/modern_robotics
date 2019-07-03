import numpy as np
from Python.modern_robotics import core as mr

#For help: help(mr.<module>)

#Problem 1: RRPR Scara Robot Test Case
#define constants
(l0, l1, l2) = (1,1,1)
(theta_1, theta_2, theta_3, theta_4) = (0, np.pi/2, -np.pi/2, 1)

m_zero = np.array([[1, 0,  0, 0],
              [ 0, 1,  0, l1+l2],
              [ 0, 0, 1, l0],
              [ 0, 0,  0, 1]])
b_list = np.array([[0, 0,  1,  -(l1+l2), 0, 0],
                  [0, 0,  1,  -l2, 0,    0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1]]).T
s_list = np.array([[0, 0,  1,  0, 0,    0],
                  [0, 0,  1,  l1, 0,    0],
                  [0, 0, 1, l1+l2, 0, 0],
                  [0, 0, 0, 0, 0, 1]]).T
theta_list = np.array([theta_1, theta_2, theta_3, theta_4])

#compute forward kinematics using 2 different methods
t04_using_fkinbody = mr.FKinBody(m_zero, b_list, theta_list)
t04_using_fkinspace = mr.FKinSpace(m_zero, s_list, theta_list)

#Problem 2:
#define constants
(l0, l1, l2, l3) = (4,3,2,1)
(theta_1, theta_2, theta_3) = (np.pi/2, 3, np.pi)

m_zero = np.array([[-1, 0,  0, 0],
                   [ 0, 1,  0, l0+l2],
                   [ 0, 0, -1, l1-l3],
                   [ 0, 0,  0, 1]])
b_2 = mr.ScrewToAxis(np.array([0, 0, -l3]),np.array([0, 1, 0]), 0.1)
b_list = np.array([[0, 0,  -1,  l2, 0, 0],
                  b_2,
                  [0, 0, 1, 0, 0, 0]]).T
s_2 = mr.ScrewToAxis(np.array([0, l0+l2, l1]),np.array([0, 1, 0]), 0.1)
s_list = np.array([[0, 0,  1,  l0, 0,    0],
                  s_2,
                  [0, 0,  -1,  -(l0+l2), 0,    0]]).T
theta_list = np.array([theta_1, theta_2, theta_3])

#compute forward kinematics using 2 different methods
t04_using_fkinbody = mr.FKinBody(m_zero, b_list, theta_list)
t04_using_fkinspace = mr.FKinSpace(m_zero, s_list, theta_list)