import time
import socket
import threading
import numpy as np

# Author: Alastair Poole
# Email: alastair.poole@strath.ac.uk
# Recv and Send UDP channels to the robot
# Each class runs a thread to send/recv coordinates
# send: to ensure the robot's receive doesn't time out, a 'filler' message is sent every 5 seconds or so
# (rob recv has a time out of 10s then kills itself, since it throws a hissy-fit if the thread runs for >15s after program finished)


class KMR_RecComms:
    def __init__(self,IP_recv,Port_recv):
        self.kin = KMRCoordinates()

        self.SocketBind = False
        self.DataRecv = False
        self.Close = False

        self.RobotPose = []
        self.RobotJoints = []
        self.KMRPose = []
        self.ProcProgress = True
        self.SubProcProgress = True

        self.FailFlag = False
        self.MissionImpossible = False

        self.BatteryLevel = -1
        self.IsCharging = False

        self.Epsilon = 1e-5

        self.OrderHash = -1

        self.rsoc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rsoc.settimeout(3)
        try:
            self.rsoc.bind((IP_recv,Port_recv))
            self.SocketBind = True
        except:
            self.SocketBind = False

    def Run(self):
        #self.GetProcessDataThread()
        self.t = threading.Thread(target = self.GetProcessDataThread)
        self.t.start()
        #print(self.t.is_alive())
        while not self.DataRecv:
            time.sleep(0.1)

    def waitOnOrder(self,num):
        while self.OrderHash!=num:
            time.sleep(0.1)

    def Close_(self):
        self.Close = True
        time.sleep(0.1)
        self.t.join()

    def GetProcessDataThread(self):
        bufs = 65535
        while True:
            if self.Close:
                return
            try:
                data = self.rsoc.recv(bufs)
                d = data.decode()
                d = d.split(' ')
                
                self.RobotJoints = np.array([float(d[k]) for k in range(7)])
                self.RobotPose = np.array([float(d[k]) for k in range(7,14)])
                self.KMRPose = np.array([float(d[k]) for k in range(14,17)])
                self.BatteryLevel = float(d[17])
                self.IsCharging = (d[18][0]=='T' or d[18][0]=='t')
                self.ProcProgress = (d[19][0]=='T' or d[19][0]=='t')
                self.SubProcProgress = (d[20][0]=='T' or d[20][0]=='t')
                self.FailFlag = (d[21][0]=='T' or d[21][0]=='t')
                self.MissionImpossible = (d[22][0]=='T' or d[22][0]=='t')
                self.OrderHash = int(d[23])
                self.DataRecv = True
            except:
                self.DataRecv = False

    def taskFin(self):
        while (not self.ProcProgress) and (not self.FailFlag):
            time.sleep(0.1)
        return self.FailFlag

    def subTaskFin(self):
        while (not self.SubProcProgress) and (not self.FailFlag):
            time.sleep(0.1)
        return self.FailFlag

    def taskFail(self):
        return self.FailFlag

    def AreWeThereKMR(self,tkmrp):
        while True:
            if ~self.DataRecv:
                return False
            diff =np.linalg.norm(tkmrp - self.KMRPose)
            if diff<self.Epsilon:
                time.sleep(2)
                return True
            time.sleep(1)

    def AreWeThereC(self,tcp):
        while True:
            if ~self.DataRecv:
                return False
            diff =np.linalg.norm(tcp - self.RobotPose)
            if diff<self.Epsilon:
                time.sleep(2)
                return True
            time.sleep(1)

    def AreWeThereJ(self,tjp):
        while True:
            if ~self.DataRecv:
                return False
            diff =np.linalg.norm(tjp - self.RobotJoints)
            if diff<self.Epsilon:
                time.sleep(2)
                return True
            time.sleep(1)


class KMR_SendComms:
    def __init__(self,IP_send,Port_send):
        self.sendsoc = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.IPs = IP_send
        self.Ps = Port_send

        self.CommandPose = []
        self.Progress_to_next = False

        self.StandbyCmd = '0 FALSE FALSE 0 0 0 0 0 0 0 -1'
        self.CurrCmd = self.StandbyCmd

        self.CloseSocket = False

        self.t = None
        self.TimeCmmdRecv = -1#time.time()
        self.TimeCmmdSent = -1#time.time()
        self.OrderNum = -1

    def Run(self):
        self.sendsoc.connect((self.IPs,self.Ps))
        self.t = threading.Thread(target = self.SendPoseThread)
        self.t.start()
        while self.TimeCmmdSent<0:
            time.sleep(0.1)

    def NextPreProgrammed(self):
        self.OrderNum = np.random.randint(1000)
        CurrCmd = '1 TRUE TRUE 0 0 0 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def Door(self,J):
        self.OrderNum = np.random.randint(1000)
        j = [str(J[i]) for i in range(7)]
        j = ' '.join(j)
        CurrCmd = '12 TRUE TRUE '+j + ' '+str(self.OrderNum)
        self.SendPose(CurrCmd)

    def MoveKMR(self,P):
        self.OrderNum = np.random.randint(1000)
        p = [str(P[i]) for i in range(3)]
        cp = ' '.join(p)
        CurrCmd = '5 TRUE TRUE '+ cp + ' 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveJ(self,J):
        self.OrderNum = np.random.randint(1000)
        j = [str(J[i]) for i in range(7)]
        j = ' '.join(j)
        CurrCmd = '9 TRUE TRUE '+j + ' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveJPlace(self,J):
        self.OrderNum = np.random.randint(1000)
        j = [str(J[i]) for i in range(7)]
        j = ' '.join(j)
        self.CurrCmd = '10 TRUE TRUE '+j + ' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveJPick(self,J):
        self.OrderNum = np.random.randint(1000)
        j = [str(J[i]) for i in range(7)]
        j = ' '.join(j)
        CurrCmd = '11 TRUE TRUE '+j + ' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveLin(self,P):
        self.OrderNum = np.random.randint(1000)
        p = [str(P[i]) for i in range(7)]
        cp = ' '.join(p)
        CurrCmd = '2 TRUE TRUE '+cp+' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveLinPlace(self,P):
        self.OrderNum = np.random.randint(1000)
        p = [str(P[i]) for i in range(7)]
        cp = ' '.join(p)
        CurrCmd = '3 TRUE TRUE '+cp+' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def MoveLinPick(self,P):
        self.OrderNum = np.random.randint(1000)
        p = [str(P[i]) for i in range(7)]
        cp = ' '.join(p)
        CurrCmd = '4 TRUE TRUE '+cp+' '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def CamCalibCommand(self):
        self.OrderNum = np.random.randint(1000)
        CurrCmd = '1 TRUE TRUE 0 0 0 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def setVialTray(self):
        self.OrderNum = np.random.randint(1000)
        CurrCmd = '6 TRUE FALSE 0 0 0 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def setVialTool(self):
        self.OrderNum = np.random.randint(1000)
        CurrCmd = '7 TRUE FALSE 0 0 0 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def setVial(self):
        self.OrderNum = np.random.randint(1000)
        CurrCmd = '8 TRUE FALSE 0 0 0 0 0 0 0 '+str(self.OrderNum)
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()

    def SendPose(self,CurrCmd):
        cmd = CurrCmd.encode('utf-8')
        self.sendsoc.sendto(cmd,(self.IPs,self.Ps))
        self.TimeCmmdSent = time.time()

    def SendPoseThread(self):
        while True:
            if self.CloseSocket:
                return
            cmd = self.CurrCmd.encode('utf-8')
            self.sendsoc.sendto(cmd,(self.IPs,self.Ps))
            self.TimeCmmdSent = time.time()
            self.CurrCmd  = self.StandbyCmd
            time.sleep(6)

    def Close_(self):
        CurrCmd = '100 TRUE TRUE 0 0 0 0 0 0 0'
        self.SendPose(CurrCmd)
        self.TimeCmmdRecv = time.time()
        self.CloseSocket = True
        time.sleep(1)
        self.t.join()

    def WaitForSend(self):# delete this function
        if True:
            return True
        while self.TimeCmmdRecv - self.TimeCmmdSent <0:
            time.sleep(0.1)
            if self.TimeCmmdRecv - self.TimeCmmdSent<-10:
                print('Comms have timed out - sending thread down.')
                self.Close()
                return false
        return True

def JPositions(fn):
    jointPositions = np.load(fn,allow_pickle = True)
    return jointPositions







