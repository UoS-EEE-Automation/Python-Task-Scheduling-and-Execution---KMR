"""
@article{park_robot_1994,
  title = {Robot sensor calibration: solving {AX}= {XB} on the {Euclidean} group},
  volume = {10},
  number = {5},
  journal = {IEEE Transactions on Robotics and Automation},
  author = {Park, Frank C and Martin, Bryan J},
  year = {1994},
  pages = {717--721}
}
@author: Myhre, Torstein A
"""
# Adapted to a class, can include more than chessboard.

import numpy as np


class HandEyeCalib:
    def __init__(self):
        self.Cam = []
        self.Rob = []
        self.A = []
        self.B = []

    def log(self,R):
        # Rotation matrix logarithm
        theta = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1.0)/2.0)
        return np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) * theta / (2*np.sin(theta))

    def invsqrt(self,mat):
        u,s,v = np.linalg.svd(mat)
        return u.dot(np.diag(1.0/np.sqrt(s))).dot(v)

    def calibrate(self):
        N = len(self.A)
        M = np.np.zeros((3,3))
        for i in range(N):
            Ra, Rb = self.A[i][0:3, 0:3], self.B[i][0:3, 0:3]
            M += np.outer(self.log(Rb), self.log(Ra))

        Rx = np.dot(np.linalg.invsqrt(np.dot(M.T, M)), M.T)

        C = np.zeros((3*N, 3))
        d = np.zeros((3*N, 1))
        for i in range(N):
            Ra,ta = self.A[i][0:3, 0:3], self.A[i][0:3, 3]
            Rb,tb = self.B[i][0:3, 0:3], self.B[i][0:3, 3]
            C[3*i:3*i+3, :] = np.identity(3) - Ra
            d[3*i:3*i+3, 0] = ta.reshape(3) - np.dot(Rx, tb)
        tx = np.dot(np.linalg.inv(np.dot(C.T, C)), np.dot(C.T, d))
        return Rx, tx.flatten()

    def NewCoords(self,robM,camM):
        if ~isinstance(robM,list):
            self.Cam.append(camM)
            self.Rob.append(robM)
            return
        for k in range(len(robM)):
            self.Rob.append(robM[k])
            self.Cam.append(camM[k])

    def ToolCalibration(self):
        n = len(self.Cam)
        if len(self.Rob)!=n:
            return False,[],[]
        for i in range(n-1):
            r0 = self.Rob[i]
            r1 = self.Rob[i+1]
            c0 = self.Cam[i]
            c1 = self.Cam[i+1]
            self.A.append(np.linalg.matmul(np.linalg.inv(r0),r1))
            self.B.append(np.linalg.matmul(c0,np.linalg.inv(c1)))
        R,t = self.calibrate()
        return True, R, t

