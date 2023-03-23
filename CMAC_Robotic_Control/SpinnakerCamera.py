import os
import PySpin
import sys
from pyzbar.pyzbar import decode
import cv2

# Author: Alastair Poole
# Email: alastair.poole@strath.ac.uk
# Interfaces the Spinnaker cam. Ensure you have th drivers installed first.
# TODO: streaming thread with image metadata.
# Finds camera pose within world-frame.

class SpinCam:
    def __init__(self):
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        num_cameras = self.cam_list.GetSize()
        if num_cameras == 0:
            self.cam_list.Clear()
            system.ReleaseInstance()
            print('Not enough cameras!')
            input('Done! Press Enter to exit...')
            return False
        self.cam = self.cam_list[0]
        self.im = []

    def getData(self):
        self.im = []
        result = False
        try:
            nodemap_tldevice = self.cam.GetTLDeviceNodeMap()
            # Initialize self.camera
            self.cam.Init()
            # Retrieve GenICam nodemap
            nodemap = self.cam.GetNodeMap()
            sNodemap = self.cam.GetTLStreamNodeMap()
            node_bufferhandling_mode = PySpin.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
            if not PySpin.IsReadable(node_bufferhandling_mode) or not PySpin.IsWritable(node_bufferhandling_mode):
                print('Unable to set stream buffer handling mode.. Aborting...')
                return False

            # Retrieve entry node from enumeration node
            node_newestonly = node_bufferhandling_mode.GetEntryByName('NewestOnly')
            if not PySpin.IsReadable(node_newestonly):
                print('Unable to set stream buffer handling mode.. Aborting...')
                return False
            node_newestonly_mode = node_newestonly.GetValue()
            node_bufferhandling_mode.SetIntValue(node_newestonly_mode)
            try:
                node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
                if not PySpin.IsReadable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                    print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
                    return False
                node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
                if not PySpin.IsReadable(node_acquisition_mode_continuous):
                    print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
                    return False
                acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
                node_acquisition_mode.SetIntValue(acquisition_mode_continuous)
                self.cam.BeginAcquisition()
                image_result = self.cam.GetNextImage(1000)
                #  Ensure image completion
                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d ...' % image_result.GetImageStatus())
                    return []
                else:
                    # Getting the image data as a numpy array
                    self.im = image_result.GetNDArray()
                image_result.Release()
            except PySpin.SpinnakerException as ex:
                print('Error: %s' % ex)
                return False
        except PySpin.SpinnakerException as ex:
            print('Error: %s' % ex)
            return False
        self.cam.EndAcquisition()
        self.cam.DeInit()
        return True

    def CloseProc(self):
        del self.cam
        self.cam_list.Clear()
        self.system.ReleaseInstance()

    def QRPose(self,PoseMat,QRsize,K,dist):
        QRvecs = np.array([[0,0,0],[QRsize,0,0],[QRsize,QRsize,0],[0,QRsize,0]])
        nCount = 0
        while nCount<10:
            if not self.GetData():
                return False,[]
            QRdata = decode(self.im)
            if len(QRdata)>0:
                edges = np.array(QRData[0].polygon)
                ret,rvecs, tvecs = cv2.solvePnP(QRvecs, edges, K, dist)
                QRpose = np.identity(4)
                QRpose[:3,:3] = cv2.Rodrigues(rvecs.flatten())[0]
                QRpose[:3,3] = tvecs.flatten()
                return True, np.matmul(PoseMat,QRPose)
            nCount += 1
        return False,[]