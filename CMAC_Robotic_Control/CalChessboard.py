import numpy
import cv2
from time import sleep

# Author: Alastair Poole
# Email: alastair.poole@strath.ac.uk
# Boiler-plate hand eye calibration procedure for the chessboard.
# TODO: Accuracy analysis of retrieved poses.

class CalChess:
    def __init__(self,Nrows,Ncols,size):
        self.Nrows = Nrows
        self.Ncols = Ncols
        self.squareSize = size
        self.pattern_size = (Nrows,Ncols)
        self.pattern_points = numpy.zeros( (numpy.prod(self.pattern_size), 3), numpy.float32 )
        self.pattern_points[:,:2] = numpy.indices(self.pattern_size).T.reshape(-1, 2)
        self.pattern_points *= size
        self.ImageList=[]
        self.K = []
        self.dist = []

    def find_corners(self,image):
        #cv2.imshow('',image)
        #cv2.waitKey(0)
        found, corners = cv2.findChessboardCorners(image, self.pattern_size)
        #term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        if found:
            self.ImageList.append(corners)
            self.imsh = image.shape[::-1]
        print('corners found: ',found)
        return found

    def get_object_pose(self):
        allP = []
        N = len(self.ImageList)
        for imageN in range(N):
            if isinstance(self.K,list):
                return False,[]
            ret, rvec, tvec = cv2.solvePnP(self.pattern_points, self.ImageList[imageN], self.K, self.dist)
            #ip,J = cv2.projectPoints(object_points,rvec,tvec, camera_matrix,dist_coeffs)
            M = numpy.identity(4)
            M[:3,:3] = cv2.Rodrigues(rvec.flatten())[0]
            M[:3,3] = tvec.flatten()
            allP.append(M)
        return True,allP

    def calibrate_lens(self):
        img_points, obj_points = [], []
        h,w = 0, 0

        for corners in self.ImageList:
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(self.pattern_points)
        camera_matrix = numpy.zeros((3,3))
        dist_coeffs = numpy.zeros(5)
    #    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h))
        #cv2.calibrateCamera(obj_points, img_points, (w,h), camera_matrix, dist_coeffs)
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, self.imsh,None,None)
        self.K = camera_matrix
        self.dist = dist_coeffs
        return ret, camera_matrix, dist_coeffs



