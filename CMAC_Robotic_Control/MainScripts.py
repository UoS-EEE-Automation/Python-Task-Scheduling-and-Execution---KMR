import numpy as np
from KMRControl import *
from SpinnakerCamera import *
from KMRKin import *
import pickle
from CalChessboard import *
from ParkMartin import *

# Author: Alastair Poole, Sebastian the Cat
# Email: alastair.poole@strath.ac.uk
# sets, retrieves and handles batches of info.
# When calling a function there's no need to initialise the send/recv ports.

class General:
    def __init__(self,rIPr,rPr,rIPs,rPs):
        self.SendEn = False
        self.RecvEn = False

        self.taskHash = {}
        self.taskHash[1] = 'Save joint position'
        self.taskHash[11] = 'Save joint position and Open gripper'
        self.taskHash[12] = 'Save joint position and Close gripper'
        self.taskHash[13] = 'Save joint position and Save QR relative pose'
        self.taskHash[2] = 'save LBR pose'
        self.taskHash[21] = 'Save LBR Pose and Open gripper'
        self.taskHash[22] = 'Save LBR Pose and Close gripper'
        self.taskHash[23] = 'Save LBR Pose and Save QR Relative Pose'
        self.taskHash[3] = 'save KMR pose'
        self.taskHash[4] = 'set Vial'
        self.taskHash[5] = 'set Vial Tool'
        self.taskHash[6] = 'set Vial Tray'
        self.taskHash[7] = 'Open/Close the door'
        self.M_corr = np.identity(4)
        self.KMR_err = np.identity(3)
        self.failFlag = False;
        try:
            with open('tasks_CMAC.pkl', 'rb') as f:
                self.tasks = pickle.load(f)
        except:
            self.tasks = {}
            print('Yet to set any tasks')
        try:
            self.TCP = np.load('ToolMat.npy', allow_pickle = True)
            self.K = np.load('IntrinsicMatrix.npy',allow_pickle = True)
            self.dist = np.load('DistortionCoeffs.npy',allow_pickle = True)
            print('Ensure camera calibration is up to date before undergoing accuracy critical tasks.')
        except:
            print('Calibrate camera before continuing.')
            self.TCP = np.identity(4)
            self.K = np.identity(3)
            self.dist = np.zeros(5)

        try:
            self.rRecv = KMR_RecComms(rIPr,rPr)
        except:
            print('Could not set up receiving port to the robot')
            return False
        try:
            self.rSend = KMR_SendComms(rIPs,rPs)
        except:
            print('Could not set up sending port to the robot')
            return False
        try:
            self.cam = SpinCam()
        except:
            print('Could not set up camera')
            return False

        self.kin = KMRCoordinates()

    def Exit(self):
        self.rRecv.Close_()
        self.rSend.Close_()

    def ExitR(self):
        self.rSend.Close()

    def startReceivingRobotData(self):
        try:
            self.rRecv.Run()
            self.RecvEn = True
            return True
        except:
            print('Could not set up receiving port to the robot')
            return False

    def startRobotControl(self):
        try:
            self.rSend.Run()
            self.SendEn = True
            return True
        except:
            print('Could not set up sending port to the robot')
            return False

    def Batchmanager(self,batch_hash):
        if not self.SendEn:
            print('Connecting to sending socket')
            success_ = self.startRobotControl()
            if not success_:
                return success_
        if not self.RecvEn:
            print('Connecting to sending socket')
            success_ = self.startReceivingRobotData()
            if not success_:
                return success_
        if batch_hash not in list(self.tasks):
            print('This task number is not allocated to a sequence - check sequencing hash value or exit.')
            return False, True
        tasksSeq = self.tasks[batch_hash]
        taskNs = list(tasksSeq)
        taskNs = [int(k) for k in taskNs]
        n = np.max(taskNs)+1

        for taskNo in range(n):
            taskInfo = tasksSeq[str(taskNo)]
            
            if taskInfo[0] == 1:
                jointPose = taskInfo[1]
                self.rSend.MoveJ(jointPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                
            elif taskInfo[0] == 11:
                jointPose = taskInfo[1]
                self.rSend.MoveJPlace(jointPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                
            elif taskInfo[0] == 12:
                jointPose = taskInfo[1]
                self.rSend.MoveJPick(jointPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                
            elif taskInfo[0] == 13:
                jointPose = taskInfo[1]
                self.rSend.MoveJ(jointPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                if self.failFlag:
                    print('Something went wrong with moving to that position.')
                    break
                # take camera image and calculate relative pose and corrections
                QRPose0 = taskInfo[2]
                QRdims = taskInfo[3]
                currP = np.matmul(self.kin.TCPTrans(self.rRecv.RobotPose),self.T) 
                success_, QRc = self.cam.QRPose(currP,QRdims,self.K,self.dist)
                if ~success:
                    print('Could not get a good view of QR code - returning to base as unable to continue.')
                    return
                self.M_corr = np.matmul(np.linalg.inv(QRc),QRPose0)

            elif taskInfo[0] == 2:
                if not np.isnan(taskInfo[-1]):
                    if np.linalg.norm(M_corr[:3,3])>taskInfo[-1]:
                        print('Positional error is too great - moving base')
                        # TODO move base, include base error calibration - x,y,theta
                robotPose = taskInfo[1]
                robotPose = self.kinCorrectPose(robotPose,M_corr)
                self.rSend.MoveLin(robotPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                result = self.rRecv.taskFin()
            
            elif taskInfo[0] == 21:
                if not np.isnan(taskInfo[-1]):
                    if np.linalg.norm(M_corr[:3,3])>taskInfo[-1]:
                        print('Positional error is too great - moving base')
                        # TODO move base, include base error calibration - x,y,theta
                robotPose = taskInfo[1]
                robotPose = self.kinCorrectPose(robotPose,M_corr)
                self.rSend.MoveLinPlace(robotPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                
            elif taskInfo[0] == 22:
                if not np.isnan(taskInfo[-1]):
                    if np.linalg.norm(M_corr[:3,3])>taskInfo[-1]:
                        print('Positional error is too great - moving base')
                        # TODO move base, include base error calibration - x,y,theta
                robotPose = taskInfo[1]
                robotPose = self.kinCorrectPose(robotPose,M_corr)
                self.rSend.MoveLinPick(robotPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.taskFail()
                
            elif taskInfo[0] == 23:
                if not np.isnan(taskInfo[-1]):
                    if np.linalg.norm(M_corr[:3,3])>taskInfo[-1]:
                        print('Positional error is too great - moving base')
                        # TODO move base, include base error calibration - x,y,theta
                robotPose = taskInfo[1]
                robotPose = self.kinCorrectPose(robotPose,M_corr)
                self.rSend.MoveLin(robotPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()
                # take camera image and calculate relative pose and corrections
                QRPose0 = taskInfo[2]
                QRdims = taskInfo[3]
                currP = np.matmul(self.kin.TCPTrans(self.rRecv.RobotPose),self.T) 
                success_, QRc = self.cam.QRPose(currP,QRdims,self.K,self.dist)
                if ~success:
                    print('Could not get a good view of QR code - returning to base as unable to continue.')
                    return
                self.M_corr = np.matmul(np.linalg.inv(QRc),QRPose0)

            elif taskInfo[0] == 3:
                kmrPose = taskInfo[1]
                kmrTP = np.array([[np.cos(kmrPose[2]),-np.sin(kmrPose[2]),kmrPose[0]],[np.sin(kmrPose[2]),np.cos(kmrPose[2]),kmrPose[1]],[0,0,1]])
                kmrcPose = self.rRecv.KMRPose
                kmrCP = np.array([[np.cos(kmrcPose[2]),-np.sin(kmrcPose[2]),kmrcPose[0]],[np.sin(kmrcPose[2]),np.cos(kmrcPose[2]),kmrcPose[1]],[0,0,1]])
                dP = np.matmul(kmrTP,np.linalg.inv(kmrCP))
                relPose = np.array([dP[0,-1],dP[1,-1],np.arctan2(dP[1,0],dP[0,0])]).flatten()
                self.rSend.MoveKMR(relPose)
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()

            elif taskInfo[0] == 4:
                self.rSend.setVial()
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()

            elif taskInfo[0] == 5:
                self.rSend.setVialTool()
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()

            elif taskInfo[0] == 6:
                self.rSend.setVialTray()
                time.sleep(0.1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.failFlag = self.rRecv.taskFin()

            elif taskInfo[0] == 7:
                p1 = taskInfo[1]# joint
                p2 = taskInfo[2]# lin - joint
                p3 = taskInfo[3]# approach/force
                p4 = taskInfo[4]# retract - joint
                p5 = taskInfo[5]# joint
                # safe pose:
                self.rSend.Door(p1)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.FailFlag = self.rRecv.subTaskFin()
                if self.FailFlag:
                    print('Issue executing motion, exiting now.')
                    return False
                # approach pose:
                self.rSend.Door(p2)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.FailFlag = self.rRecv.subTaskFin()
                if self.FailFlag:
                    print('Issue executing motion, exiting now.')
                    return False
                # close door:
                self.rSend.Door(p3)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.FailFlag = self.rRecv.subTaskFin()
                if self.FailFlag:
                    print('Issue executing motion, exiting now.')
                    return False
                # retract:
                self.rSend.Door(p4)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.FailFlag = self.rRecv.subTaskFin()
                if self.FailFlag:
                    print('Issue executing motion, exiting now.')
                    return False
                # go to safe pose0
                self.rSend.Door(p5)
                self.rRecv.waitOnOrder(self.rSend.OrderNum)
                self.FailFlag = self.rRecv.subTaskFin()
                if self.FailFlag:
                    print('Issue executing motion, exiting now.')
                    return False
                self.rRecv.taskFin()

            if self.failFlag:
                print('Batch motion process has failed, and are exiting the process')
                return False
        return True

    def SetTaskSequencing(self):
        if not self.RecvEn:
            print('Connecting to sending socket')
            success_ = self.startReceivingRobotData()
            if not success_:
                return success_
        while True:
            taskKey = int(input('Task identifier number:'))
            keyList = list(self.tasks)
            if taskKey in keyList:
                curTaskDict = self.tasks[taskKey]
                print('Retrieved old task sequence for key: ',taskKey)
            else:
                curTaskDict = {}
                print('Assigned new task sequence')
            stayOnTask = True
            curKeys = list(curTaskDict)
            if len(curKeys)>0:
                print('Current task identifiers assigned to this sequence:')
                for k in curKeys:
                    print(k,'---',self.taskHash[curTaskDict[k][0]])
            while stayOnTask:
                taskN = input('Input specific task number (or "n" for setting all in sequence)')
                if taskN == 'n':
                    taskNh = 0
                    while stayOnTask:
                        cont = input('Press enter when the robot is in the correct pose')
                        print('press:')
                        print('1 -- Save joint position')
                        print('11 -- Save joint position and Open gripper')
                        print('12 -- Save joint position and Close gripper')
                        print('2 -- save LBR pose')
                        print('21 -- Save LBR Pose and Open gripper')
                        print('22 -- Save LBR Pose and Close gripper')
                        print('3 -- save KMR pose')
                        print('4 -- set Vial')
                        print('5 -- set Vial Tool')
                        print('6 -- set Vial Tray')
                        print('7 -- Open/close a door')
                        print('100 -- abort')
                        taskNo = input('Task Number: ')
                        if taskNo == '100':
                            eg = input('Abort all motion setting? (y/n)')
                            if eg == 'y':
                                return
                            eg = input('Abort this motion setting? (y/n)')
                            if eg =='y':
                                stayOnTask = False
                        if stayOnTask:
                            success = True
                            if taskNo[0] == '1':
                                addInfo = [self.rRecv.RobotJoints]
                                if taskNo == '13':
                                    RobPose = self.kin.FKin(addInfo[0])
                                    QRdims = input('Put in the QR code dimensions (mm): ')
                                    QRdims = float(QRdims)
                                    success,QRPose = self.cam.QRPose(np.matmul(RobPose,self.TCP),QRdims,self.K,self.dist)
                                    if not success:
                                        print('Could not get a clear view of the QR code - consider re-trying')
                                    else:
                                        addInfo = [self.rRecv.RobotJoints,QRPose,QRdims]

                            elif taskNo[0] == '2':
                                safetyRad = input('Safety radius of task (n for none):')
                                if safetyRad == 'n':
                                    safetyRad = 'NaN'
                                addInfo = [self.Revc.RobotPose,float(safetyRad)]
                                if taskNo == '23':
                                    QRdims = input('Put in the QR code dimensions (mm): ')
                                    QRdims = float(QRdims)
                                    RobPose = self.kin.TCPTrans(addInfo[0])
                                    success,QRPose = self.cam.QRPose(np.matmul(RobPose,self.TCP),QRdims,self.K,self.dist)
                                    if not success:
                                        print('Could not get a clear view of the QR code - consider re-trying')
                                    else:
                                        addInfo = [self.rRecv.RobotJoints,QRPose,QRdims,float(safetyRad)]

                            elif taskNo[0] == '3':
                                addInfo = [self.rRecv.KMRPose]

                            elif taskNo[0] == '7':
                                jp1 = self.rRecv.RobotJoints
                                print('First joint position set in place for safe motion, now the initial Cartesian position')
                                _ = input('Press enter when at the first position.')
                                cp1 =  self.rRecv.RobotJoints
                                print('First joint position and Cartesian approach position set in place, now the door-closed Cartesian position.')
                                _ = input('Press enter when at the pose.')
                                cp2 =  self.rRecv.RobotPose
                                print('Close/Open task positions set, now the safe Cartesian extraction.')
                                _ = input('Press enter when at the position.')
                                cp3 = self.rRecv.RobotPose
                                ans_ = input('End sequence with the starting joint position? (y/n) ')
                                if ans_=='y':
                                    jp2 = np.copy(jp1)
                                else:
                                    _ = input('Press enter when at the desired joint position.')
                                    jp2 = self.rRcv.RobotJoints
                                addInfo = [jp1,cp1,cp2,cp3,jp2]
                            else:
                                addInfo = [None]
                            if success:
                                print('Adding task: ' + self.taskHash[int(taskNo)])
                                addInfo.insert(0,int(taskNo))
                                curTaskDict[taskNh] = addInfo
                                self.tasks[taskKey] = curTaskDict
                                taskNh+=1
                                print('Next task number: ',taskNh)
                            else:
                                print('Task assignment not successful and so not added to the sequence.')                            
                else:
                    cont = input('Press enter when the robot is in the correct pose')
                    print('press:')
                    print('1 -- Save joint position')
                    print('11 -- Save joint position and Open gripper')
                    print('12 -- Save joint position and Close gripper')
                    print('2 -- save LBR pose')
                    print('21 -- Save LBR Pose and Open gripper')
                    print('22 -- Save LBR Pose and Close gripper')
                    print('3 -- save KMR pose')
                    print('4 -- set Vial')
                    print('5 -- set Vial Tool')
                    print('6 -- set Vial Tray')
                    print('7 -- Open/close door')
                    print('100 -- abort')
                    taskNo = input('Task Number: ')
                    if taskNo == '100':
                        eg = input('Abort all motion setting? (y/n)')
                        if eg == 'y':
                            return
                        eg = input('Abort this motion setting? (y/n)')
                        if eg =='y':
                            stayOnTask = False
                    if stayOnTask:
                        success = True
                        if taskNo[0] == '1':
                            addInfo = [self.rRecv.RobotJoints]
                            if taskNo == '13':
                                RobPose = self.kin.FKin(addInfo[0])
                                QRdims = input('Put in the QR code dimensions (mm): ')
                                QRdims = float(QRdims)
                                success,QRPose = self.cam.QRPose(np.matmul(RobPose,self.TCP),QRdims,self.K,self.dist)
                                if not success:
                                    print('Could not get a clear view of the QR code - consider re-trying')
                                else:
                                    addInfo = [self.rRecv.RobotJoints,QRPose,QRdims]
                        elif taskNo[0] == '2':
                            safetyRad = input('Safety radius of task (n for none):')
                            if safetyRad == 'n':
                                safetyRad = 'NaN'
                            addInfo = [self.Revc.RobotPose,float(safetyRad)]
                            if taskNo == '23':
                                QRdims = input('Put in the QR code dimensions (mm): ')
                                QRdims = float(QRdims)
                                RobPose = self.kin.TCPTrans(addInfo[0])
                                success,QRPose = self.cam.QRPose(np.matmul(RobPose,self.TCP),QRdims,self.K,self.dist)
                                if not success:
                                    print('Could not get a clear view of the QR code - consider re-trying')
                                else:
                                    addInfo = [self.rRecv.RobotJoints,QRPose,QRdims,float(safetyRad)]
                        elif taskNo[0] == '3':
                            addInfo = [self.rRecv.KMRPose]

                        elif taskNo[0] == '7':
                            jp1 = self.rRecv.RobotJoints
                            print('First joint position set in place for safe motion, now the initial Cartesian position')
                            _ = input('Press enter when at the first position.')
                            cp1 =  self.rRecv.RobotJoints
                            print('First joint position and Cartesian approach position set in place, now the door-closed Cartesian position.')
                            _ = input('Press enter when at the pose.')
                            cp2 =  self.rRecv.RobotPose
                            print('Close/Open task positions set, now the safe Cartesian extraction.')
                            _ = input('Press enter when at the position.')
                            cp3 = self.rRecv.RobotPose
                            ans_ = input('End sequence with the starting joint position? (y/n) ')
                            if ans_=='y':
                                jp2 = np.copy(jp1)
                            else:
                                _ = input('Press enter when at the desired joint position.')
                                jp2 = self.rRcv.RobotJoints
                            addInfo = [jp1,cp1,cp2,cp3,jp2]
                        else:
                            addInfo = [None]
                        if success:
                            print('Adding task: ' + self.taskHash[int(taskNo)])
                            addInfo.insert(0,int(taskNo))
                            curTaskDict[taskN] = addInfo
                            self.tasks[taskKey] = curTaskDict
                        else:
                            print('Task assignment not successful and so not added to the sequence.')

                exit_ = input('Keep Inputting task sequences? (y/n)')
                if exit_ == 'n':
                    ans_ = input('Save updated task sequence? (y/n)')
                    if ans_=='y':
                        with open('tasks_CMAC.pkl', 'wb') as f:
                            pickle.dump(self.tasks, f)
                    else:
                        print('Not saving - I hope you meant to press that key...')
                    return

    def CameraCalibration(self,Nrow,Ncols,size):
        if not self.RecvEn:
            print('Connecting to sending socket')
            success_ = self.startReceivingRobotData()
            if not success_:
                return success_

        if not self.SendEn:
            print('Connecting to sending socket')
            success_ = self.startRobotControl()
            if not success_:
                return success_

        
        ChessClass = CalChess(Nrow,Ncols,size)
        RobL = []
        ProcFin = False
        while not ProcFin:
            # next position is a go-go
            self.rSend.NextPreProgrammed()
            time.sleep(0.1)
            self.rRecv.waitOnOrder(self.rSend.OrderNum)
            result = self.rRecv.subTaskFin()

            if result:
                print('something went wrong while getting to target joint position number: ')
                break
            result = self.cam.getData()
            if isinstance(result,bool):
                if not result:
                    print('something went wrong while collecting image number: ')
                    break
            else:
                time.sleep(1)
                result = self.cam.getData()
                if isinstance(result,bool):
                    if not result:
                        print('something went wrong while collecting image number: ')
                        break
                else:
                    print('something went wrong while collecting image number: ')
                    break
            P = self.rRecv.RobotPose
            if isinstance(P,list):#~result:
                print('something went wrong while collecting robot position number: ',k)
                break

            result = ChessClass.find_corners(self.cam.im)
            if result:
                M = self.kin.TCPTrans(P)
                RobL.append(M)
            else:
                print('couldnt find corners on this image')
            time.sleep(1)
            ProcFin = self.rRecv.taskFin()

        print('Calibrating camera parameters')
        _,K,dist = ChessClass.calibrate_lens()
        print('Camera Matrix: ')
        print(K)
        ans_ , CamL = ChessClass.get_object_pose()
        if not ans_:
            print('Something went wrong in the inrinsic calibration process.')
            return
        toolcalib = HandEyeCalib()
        toolcalib.NewCoords(RobL,CamL)
        R,t = toolcalib.ToolCalibration()
        T = np.identity(4)
        T[:3,:3] = R
        T[:3,3] = t

        np.save('ToolMat', T)
        np.save('IntrinsicMatrix',K)
        np.save('DistortionCoeffs',dist)

        self.K = K
        self.camTCP = R

    def RunBatches(self):
        if len(list(self.tasks))==0:
            print('you have got to set tasks first.')
            return False
        while True:
            inp = input('Input task number (n for exit): ')
            if inp=='n':
                return True
            try:
                inp = int(inp)
            except:
                print('That is not an integer, numpty.')
                break

            if inp not in list(self.tasks):
                print('Define task ' + str(inp)+' first.')
                break
            success_ = self.Batchmanager(inp)
            if ~success_:
                print('That did not work for some reason. Now doing something like emailing a lab technician to come solve the problem...')
            ex = input('Escape process?')
            if ex=='y':
                return
