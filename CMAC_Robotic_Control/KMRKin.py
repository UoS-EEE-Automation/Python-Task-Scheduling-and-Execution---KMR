import numpy as np

# Author: Alastair Poole
# Email: alastair.poole@strath.ac.uk
# Standard kinematics functions.
# Converts between the deeply flawed Tait-Bryan convention used by KUKA and the proper SE(3) representation. 


class KMRCoordinates:
    def __init__(self,TCP = np.identity(4)):
        self.TCP = TCP
        self.d = np.array([0,0,420,0,400,0,0])
        self.a = np.zeros(7)
        self.alpha = np.array([0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,np.pi/2])

    def Rz(self,a):
        rz = np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        return rz

    def Ry(self,a):
        ry = np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        return ry

    def Rx(self,a):
        rx = np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
        return rx

    def TCPTrans(self,P):
        t = np.identity(4)
        t[:3,:3] = np.matmul(self.Rz(P[3]),np.matmul(self.Ry(P[4]),self.Rx(P[5])))
        t[:3,3] = P[:3]
        t = np.matmul(t,self.TCP)
        return t

    def M2V(self,M):
        v = np.zeros(6)
        v[:3] = M[:3,3].flatten()
        a = np.arctan2(M[1,0],M[0,0])
        b = np.arcsin(-M[2,0])
        c = np.arctan2(M[2,1],M[2,2])
        v[3] = a
        v[4] = b
        v[5] = c
        return v

    def FindCorrections(self,M0,P_c,T,QRT):
        M_c = self.TCPTrans(P_c)
        M_c = np.matmul(np.matmul(M_c,T),QRT)
        return np.matmul(np.linalg.inv(M_c),M0)

    def FindCorrectionsJ(self,M0,P_c,T,QRT):
        M_c = self.FKin(P_c)
        M_c = np.matmul(np.matmul(M_c,T),QRT)
        return np.matmul(np.linalg.inv(M_c),M0)

    def CorrectPose(self,P0,Mcorr):
        M0 = self.TCPTrans(P0)
        M1 = np.matmul(Mcorr,M0)
        P1 = self.M2V(M1)
        return P1

    def DH(self,angle,n):
        Z = np.identity(4)
        X = np.identity(4)
        Z[:2,:2] = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        Z[2,3] = self.d[n]
        X[1:3,1:3] = np.array([[np.cos(self.alpha[n]),-np.sin(self.alpha[n])],[np.sin(self.alpha[n]),np.cos(self.alpha[n])]])
        X[0,3] = self.a[n]
        return np.matmul(Z,X)

    def FKin(self,A):
        # This needs major fixes before deployment - find the right DH parameters.
        Trans = np.identity(4)
        for k in range(7):
            Trans = np.matmul(Trans,self.DH(A[k],k))
        return Trans


