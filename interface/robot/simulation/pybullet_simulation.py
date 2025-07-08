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
import os
from colorama import init, Fore, Style

# Initialize colorama for colored terminal output
init(autoreset=True)
urdfPath = {
"lite3":    "/../../../third_party/URDF_model/Lite3/Lite3_urdf/urdf/Lite3.urdf",
}

initJointPos = {
"lite3":    [0, -1.35453, 2.54948]*4,
}

class PyBulletSimulation:

    def __init__(self, robot_name, local_port=20001, ctrl_ip="127.0.0.1", ctrl_port=30010) -> None:
        self.localPort = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.server.settimeout(5)
        self.ip = ctrl_ip
        self.ctrlAddr = (ctrl_ip, ctrl_port)
        self.robotName = robot_name

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pd.getDataPath()) # 设置pybullet_data的文件路径
        self.terrain=p.loadURDF("plane.urdf") 
        p.changeDynamics(self.terrain, -1, lateralFriction=1)
        zInit = 0.42 
        startPos = [0.0, 0, zInit]
        currentFilePath = os.path.abspath(__file__)
        currentDirectory = os.path.dirname(currentFilePath)
        robotUrdfPath = currentDirectory + urdfPath[self.robotName]
        urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.robot = p.loadURDF(robotUrdfPath, startPos, flags=urdfFlags, useFixedBase=False)
        
        self.numOfJoints = p.getNumJoints(self.robot)
        self.jointIdxList = []
        footNumList=[3, 7, 11, 15]
        p.changeDynamics(self.robot, -1, linearDamping=0.0, angularDamping=0.0)
        for j in range(self.numOfJoints):
            jointInfo = p.getJointInfo(self.robot, j)
            jointName = jointInfo[1].decode("UTF-8")
            lower = jointInfo[8]
            upper = jointInfo[9]
            p.changeDynamics(self.robot, j, linearDamping=0.0, angularDamping=0.0)
            if j in footNumList:
                p.changeDynamics(self.robot, j, lateralFriction=0.8, spinningFriction=0.03*0.8)
            if jointInfo[2]==p.JOINT_REVOLUTE:
                self.jointIdxList.append(j)
                p.resetJointState(self.robot, j, targetValue=initJointPos[self.robotName][len(self.jointIdxList)-1])
                p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)
                # print(j, " : ", jointName, " qIndex : ", jointInfo[3], " uIndex : ", jointInfo[4], "lower : ", lower, 'upper : ', upper)
        self.dofNum=len(self.jointIdxList)
        self.inputTorque = [0.]*self.dofNum
        self.lastBaseVelWorld = np.zeros((3, 1))
        self.kpCmd = np.zeros((self.dofNum, 1))
        self.kdCmd = np.zeros((self.dofNum, 1))
        self.jointPosCmd = np.zeros((self.dofNum, 1))
        self.jointVelCmd = np.zeros((self.dofNum, 1))
        self.tauCmd = np.zeros((self.dofNum, 1))
        # print(self.jointIdxList)
        # print(self.jointPosCmd)
        
        self.last_print_time = 0  # Track last print time
        
    def print_debug_info(self):
        """Consolidated function to print debug information with colors and aligned formatting."""
        # Format arrays with 2 decimal places and fixed width
        def format_array(arr):
            return "[" + ", ".join(f"{x:6.2f}" for x in arr) + "]"

        print(f"{Fore.CYAN}=== [Debug Info] ==={Style.RESET_ALL}")
        print(f"{Fore.GREEN}[IMU] RPY        :{Style.RESET_ALL} {format_array(self.baseRpy.flatten())}")
        print(f"{Fore.GREEN}[IMU] Omega      :{Style.RESET_ALL} {format_array(self.baseOmega.flatten())}")
        print(f"{Fore.GREEN}[IMU] Acc_body   :{Style.RESET_ALL} {format_array(self.baseAcc.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Position  :{Style.RESET_ALL} {format_array(self.jointPos.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Velocity  :{Style.RESET_ALL} {format_array(self.jointVel.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Torque    :{Style.RESET_ALL} {format_array(self.jointTau.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Pos:{Style.RESET_ALL} {format_array(self.jointPosCmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Pos:{Style.RESET_ALL} {format_array(self.jointPos.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Vel:{Style.RESET_ALL} {format_array(self.jointVelCmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Vel:{Style.RESET_ALL} {format_array(self.jointVel.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kp Term   :{Style.RESET_ALL} {format_array(self.kpCmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kd Term   :{Style.RESET_ALL} {format_array(self.kdCmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] FF Tau    :{Style.RESET_ALL} {format_array(self.tauCmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Final Torq:{Style.RESET_ALL} {format_array(self.inputTorque.T.flatten())}")
        print(f"{Fore.YELLOW}[IMU] jointTau    :{Style.RESET_ALL} {format_array(self.jointTau.flatten())}")
        print(f"{Fore.CYAN}==================={Style.RESET_ALL}")
            

    def startSimulation(self):
        runCnt = 0
        p.setTimeStep(0.001)
        while True:
            startTime = time.time()
            runCnt += 1
            self.timestamp = runCnt * 0.001
            self.getJointMessage()
            self.getImuMessage()
            self.sendRobotData()
            self.setJointCmd(self.kpCmd, self.jointPosCmd, self.kdCmd, self.jointVelCmd, self.tauCmd)
            p.setJointMotorControlArray(self.robot, self.jointIdxList, 
                                        controlMode=p.TORQUE_CONTROL, 
                                        forces=self.inputTorque.reshape(self.dofNum).tolist())
            p.stepSimulation()

            # Print at 0.5 Hz (every 2 seconds)
            current_time = time.time()
            if current_time - self.last_print_time >= 2.0:
                self.print_debug_info()
                self.last_print_time = current_time

            costTime = time.time() - startTime
            if costTime < 0.001:
                time.sleep(0.001 - costTime)


    def getImuMessage(self):
        _, baseQuat = p.getBasePositionAndOrientation(self.robot)
        baseRpy = p.getEulerFromQuaternion(baseQuat)
        baseVelWorld, baseOmegaWorld = p.getBaseVelocity(self.robot)
        rotMat = p.getMatrixFromQuaternion(baseQuat)
        rotMat = np.array(rotMat).reshape(3, 3)
        self.baseRpy = np.array(baseRpy).reshape(3, 1)
        self.baseOmega = rotMat.T @ np.array(baseOmegaWorld).reshape(3, 1)
        baseVelWorld = np.array(baseVelWorld).reshape(3, 1)
        baseAccWorld = (baseVelWorld - self.lastBaseVelWorld) / 0.001
        self.lastBaseVelWorld = baseVelWorld
        baseAccWorld[2, 0] = baseAccWorld[2, 0] + 9.81
        self.baseAcc = rotMat.T @ baseAccWorld


    def getJointMessage(self):
        jointStates = p.getJointStates(self.robot, self.jointIdxList)
        self.jointPos = np.array([jointStates[i][0] for i in range(self.dofNum)]).reshape(self.dofNum, 1)
        self.jointVel = np.array([jointStates[i][1] for i in range(self.dofNum)]).reshape(self.dofNum, 1)
        self.jointTau = np.array([jointStates[i][3] for i in range(self.dofNum)]).reshape(self.dofNum, 1)




    def setJointCmd(self, kp, targetPos, kd, targetVel, tau):
        self.inputTorque = np.multiply(kp, (targetPos - self.jointPos)) \
                         + np.multiply(kd, (targetVel - self.jointVel)) \
                         + tau


    def sendRobotData(self):
        timestamp = np.array([self.timestamp]).flatten()
        rpy = self.baseRpy.flatten()
        acc = self.baseAcc.flatten()
        omg = self.baseOmega.flatten()
        jointPos = self.jointPos.flatten()
        jointVel = self.jointVel.flatten()
        jointTau = self.jointTau.flatten()
        combineData = np.concatenate((timestamp, rpy, acc, omg, jointPos, jointVel, jointTau))
        formatString = f'1d{len(combineData)-1}f'
        data = struct.pack(formatString, *combineData)
        if self.server:
            try:
                self.server.sendto(data, self.ctrlAddr)
            except socket.error as ex:
                print(f"[UDP Error] Failed to send: {ex}")
        
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
            
            # print(f"[UDP] Received {len(data)} bytes from {addr}")
            # print(f"[Recv] Kp:\n{self.kpCmd.T}")
            # print(f"[Recv] Kd:\n{self.kdCmd.T}")
            # print(f"[Recv] Target Pos:\n{self.jointPosCmd.T}")
            # print(f"[Recv] Target Vel:\n{self.jointVelCmd.T}")
            # print(f"[Recv] Tau:\n{self.tauCmd.T}")
            # print(self.kpCmd)
        udpSocket.close()
        


if __name__ == '__main__':
    ps = PyBulletSimulation("lite3")
    receiveThread = threading.Thread(target=ps.receiveJointCmd)
    receiveThread.start()
    ps.startSimulation()
 




