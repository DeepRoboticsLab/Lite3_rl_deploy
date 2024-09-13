"""
 * @file pybullet_simulation.py
 * @brief simulation in pybullet
 * @author mazunwang
 * @version 1.0
 * @date 2024-09-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
"""
import pybullet as p
import numpy as np
import pybullet_data as pd
import socket
import struct
import threading
import time

urdfPath = {
"lite3"    : "../../../third_party/URDF_model/lite3_urdf/lite3_pybullet/Lite3/urdf/Lite3.urdf",
}

class PyBulletSimulation:
    def __init__(self, robot_name, urdf_path, local_port=20001, ctrl_ip="127.0.0.1", ctrl_port=30010) -> None:
        self.localPort = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.server.settimeout(5)
        self.ip = ctrl_ip
        self.ctrlAddr = (ctrl_ip, ctrl_port)

        self.robot_name = robot_name
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pd.getDataPath()) # 设置pybullet_data的文件路径
        p.loadURDF("plane.urdf") 
        zInit = 0.45 
        startPos = [0.0, 0, zInit]
        urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.robot = p.loadURDF(urdf_path, startPos, flags=urdfFlags, useFixedBase=False)
        
        self.numOfJoints = p.getNumJoints(self.robot)
        self.jointIdxList = []
        
        for j in range(self.numOfJoints):
            jointInfo = p.getJointInfo(self.robot, j)
            jointName = jointInfo[1].decode("UTF-8")
            lower = jointInfo[8]
            upper = jointInfo[9]
            
            if jointInfo[2]==p.JOINT_REVOLUTE:
                self.jointIdxList.append(j)
                p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)
                print(j, " : ", jointName, " qIndex : ", jointInfo[3], " uIndex : ", jointInfo[4], "lower : ", lower, 'upper : ', upper)
        self.dofNum=len(self.jointIdxList)
        self.inputTorque = [0.]*self.dofNum
        self.lastBaseVelWorld = np.zeros((3, 1))
        self.kpCmd = np.zeros((self.dofNum, 1))
        self.kdCmd = np.zeros((self.dofNum, 1))
        self.jointPosCmd = np.zeros((self.dofNum, 1))
        self.jointVelCmd = np.zeros((self.dofNum, 1))
        self.tauCmd = np.zeros((self.dofNum, 1))
        print(self.jointIdxList)
        # print(self.jointPosCmd)
            

    def startSimulation(self):
        runCnt = 0
        p.setTimeStep(0.001)
        while True:
            startTime = time.time()
            runCnt+=1
            self.timestamp=runCnt*0.001
            self.getJointMessage()
            self.getImuMessage()
            self.sendRobotData()
            self.setJointCmd(self.kpCmd, self.jointPosCmd, self.kdCmd, self.jointVelCmd, self.tauCmd)
            p.setJointMotorControlArray(self.robot, self.jointIdxList, 
                                        controlMode=p.TORQUE_CONTROL, 
                                        forces=self.inputTorque.reshape(self.dofNum).tolist())
            p.stepSimulation()
            endTime = time.time()
            time.sleep(0.001-(endTime-startTime))


    def getImuMessage(self):
        _, baseQuat = p.getBasePositionAndOrientation(self.robot)
        baseRpy = p.getEulerFromQuaternion(baseQuat)
        baseVelWorld, baseOmegaWorld = p.getBaseVelocity(self.robot)
        rotMat = p.getMatrixFromQuaternion(baseQuat)
        rotMat = np.array(rotMat).reshape(3, 3)
        self.baseRpy = np.array(baseRpy).reshape(3, 1)
        self.baseOmega = rotMat.T@np.array(baseOmegaWorld).reshape(3, 1)
        baseVelWorld = np.array(baseVelWorld).reshape(3, 1)
        baseAccWorld = (baseVelWorld - self.lastBaseVelWorld)/0.001
        self.lastBaseVelWorld = baseVelWorld
        baseAccWorld[2, 0] = baseAccWorld[2, 0] + 9.81
        self.baseAcc = rotMat.T@baseAccWorld

    def getJointMessage(self):
        jointStates = p.getJointStates(self.robot, self.jointIdxList)
        self.jointPos = np.array([jointStates[i][0] for i in range(self.dofNum)]).reshape(self.dofNum, 1)
        self.jointVel = np.array([jointStates[i][1] for i in range(self.dofNum)]).reshape(self.dofNum, 1)
        self.jointTau = np.array([jointStates[i][3] for i in range(self.dofNum)]).reshape(self.dofNum, 1)

    def setJointCmd(self, kp, targetPos, kd, targetVel, tau):
        self.inputTorque = np.multiply(kp, (targetPos-self.jointPos)) \
                         + np.multiply(kd, (targetVel-self.jointVel))\
                         + tau
        

    def sendRobotData(self):
        timestamp = np.array([self.timestamp]).flatten()
        rpy = self.baseRpy.flatten()
        acc= self.baseAcc.flatten()
        omg = self.baseOmega.flatten()
        jointPos = self.jointPos.flatten()
        jointVel = self.jointVel.flatten()
        jointTau = self.jointTau.flatten()
        combineData = np.concatenate((timestamp, rpy, acc, omg, jointPos, jointVel, jointTau))
        formatString = f'{len(combineData)}f'
        data = struct.pack(formatString, *combineData)
        # self.comm_lock.acquire()
        if self.server:
            try:
                self.server.sendto(data, self.ctrlAddr)
            except socket.error as ex:
                print(ex)
        # self.comm_lock.release()
        
    def receiveJointCmd(self):
        udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udpSocket.bind(('127.0.0.1', self.localPort))  # 绑定本地IP和端口

        while True:
            data, addr = udpSocket.recvfrom(1024)  # 接收数据
            kp = struct.unpack('12f', data[0:12*4])
            targetPos = struct.unpack('12f', data[12*4:24*4])
            kd = struct.unpack('12f', data[24*4:36*4])
            targetVel = struct.unpack('12f', data[36*4:48*4])
            tau = struct.unpack('12f', data[48*4:60*4])
            self.kpCmd = np.array(kp).reshape(12, 1)
            self.kdCmd = np.array(kd).reshape(12, 1)
            self.jointPosCmd = np.array(targetPos).reshape(12, 1)
            self.jointVelCmd = np.array(targetVel).reshape(12, 1)
            self.tauCmd = np.array(tau).reshape(12, 1)
            # print(self.kpCmd)
        udpSocket.close()


if __name__ == '__main__':
    ps = PyBulletSimulation("lite3", urdfPath["lite3"])
    receiveThread = threading.Thread(target=ps.receiveJointCmd)
    receiveThread.start()
    ps.startSimulation()
 




