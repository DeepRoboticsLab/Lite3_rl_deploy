"""
 * @file forward_kinematic.py
 * @brief simulation in pybullet
 * @author mazunwang
 * @version 1.0
 * @date 2024-10-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import pybullet as p
import numpy as np
import pybullet_data as pd
from math import *

urdfFileName = "../../../third_party/URDF_model/lite3_urdf/lite3_pybullet/Lite3/urdf/Lite3.urdf"
PI = np.pi

def rotX(roll):
    res = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    return res

def rotY(pitch):
    res = np.array([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    return res

def rotZ(yaw):
    res = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    return res

def legForwardKinematic(angle, leg):
    """calculate lite3 leg forward kinematic

    Args:
        angle: joint positon of 3 joints in one leg
        leg (int): 0:FL, 1:FR, 2:HL, 3:HR
		
	Returns: 
        [vec3]:foot position in base frame
    """
    lHip = 0.09735
    lThigh = 0.2
    lShank = 0.21012
    bodyLenX = 0.1745
    bodyLenY = 0.062
    bodyLenZ = 0
    bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape((3, 1))
    
    hipVec = np.array([0, lHip, 0])
    if leg%2==1:
        hipVec = np.array([0, -lHip, 0])
        bodyVec[1, 0] = -bodyLenY

    if leg/2==1:
        bodyVec[0, 0] = -bodyLenX
    hipVec = hipVec.reshape((3, 1))
    thighVec = np.array([0, 0, -lThigh]).reshape((3, 1))
    shankVec = np.array([0, 0, -lShank]).reshape((3, 1))
    rotHipX = rotX(-angle[0])
    rotHipY = rotY(-angle[1])
    rotKnee = rotY(-angle[2])
    footPos = rotHipX@(hipVec+rotHipY@(thighVec+rotKnee@shankVec))
    return footPos+bodyVec

def legInverseKinematic(pos, leg):
	"""calculate lite3 leg inverse kinematic

	Args:
		pos (vec3): foot position in base frame
		leg (int) : 0:FL, 1:FR, 2:HL, 3:HR
    
    Returns: 
        [vec3]:joint angle
	"""
	lHip = 0.09735
	lThigh = 0.2
	lShank = 0.21012
	bodyLenX = 0.1745
	bodyLenY = 0.062
	bodyLenZ = 0
	bodyVec = np.array([bodyLenX, bodyLenY, bodyLenZ]).reshape((3, 1))
	if leg%2==1:
		bodyVec[1, 0] = -bodyLenY
		lHip = -0.09735
	if leg/2==1:
		bodyVec[0, 0] = -bodyLenX
	legPos = pos-bodyVec 
	px, py, pz = legPos[0, 0], legPos[1, 0], legPos[2, 0]
	thetaHipx = -acos(lHip/sqrt(py**2+pz**2))-atan2(pz, py)
	l0=sqrt(px**2+py**2+pz**2-lHip**2)
	thetaHipy = asin(px/l0)-acos((lThigh**2+l0**2-lShank**2)/(2*l0*lThigh))
	thetaKnee = PI-acos((lShank**2+lThigh**2-l0**2)/(2*lShank*lThigh))
	return np.array([thetaHipx, thetaHipy, thetaKnee]).reshape((3, 1))
		
	
	
	
p.connect(p.GUI) # 连接到仿真服务器
p.setGravity(0, 0, -9.81) # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath()) # 设置pybullet_data的文件路径
 
# 加载地面
floor = p.loadURDF("plane.urdf") 
zInit = 0.5
startPos = [0.0, 0, zInit]
urdfFlags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | p.URDF_USE_INERTIA_FROM_FILE
robot = p.loadURDF(urdfFileName, startPos, flags=urdfFlags, useFixedBase=True)
 
# 获取关节数量
numOfJoints = p.getNumJoints(robot)
jointPos = []
print("numOfJoints:  ", numOfJoints)

rollId = p.addUserDebugParameter(" roll", -PI/2.0, PI/2.0, 0)
pitchId = p.addUserDebugParameter(" pitch", -PI/2.0, PI/2.0, 0)
yawId = p.addUserDebugParameter(" yaw", -PI/2.0, PI/2.0, 0)
xWorld = p.addUserDebugParameter(" X", -1, 1, 0)
yWorld = p.addUserDebugParameter(" Y", -1, 1, 0)
zWorld = p.addUserDebugParameter(" Z", 0.0, 2, zInit)
 
# 设置初始关节角度
jointIndexList = []
for j in range(numOfJoints):
	jointInfo = p.getJointInfo(robot, j)
	jointName = jointInfo[1].decode("UTF-8")
	lower = jointInfo[8]
	upper = jointInfo[9]
	if upper < lower:
		upper = PI
		lower = -PI
	default = (lower+upper)/2.
	if lower*upper <= 0:
		default=0
	if jointInfo[2]==p.JOINT_REVOLUTE:
		jointIndexList.append(j)
		jointPos.append(p.addUserDebugParameter(' '+jointName, lower, upper, default))
	print(j, " : ", jointName, " qIndex : ", jointInfo[3], " uIndex : ", jointInfo[4], "lower : ", lower, 'upper : ', upper)

dt = 1./1000.
jointForceLimit = 200

numOfRevolute = len(jointIndexList)
jointPosVec = [0]*numOfRevolute
p.setTimeStep(dt)
# 进行仿真
while True:
	# p.addUserDebugLine([0, 0, 0], [1, 1, 1], lineColorRGB=[1, 0, 0])
	roll = p.readUserDebugParameter(rollId)
	pitch = p.readUserDebugParameter(pitchId)
	yaw = p.readUserDebugParameter(yawId)
	xSet = p.readUserDebugParameter(xWorld)
	ySet = p.readUserDebugParameter(yWorld)
	zSet = p.readUserDebugParameter(zWorld)
	quat = p.getQuaternionFromEuler([roll, pitch, yaw])
	p.resetBasePositionAndOrientation(robot, [xSet, ySet, zSet], quat)
	for i in range(numOfRevolute):
		jointPosVec[i] = p.readUserDebugParameter(jointPos[i])
		p.resetJointState(robot, jointIndexList[i], jointPosVec[i])#关节限制不存在了
	legNum=1
	jointPosInput=jointPosVec[legNum*3:legNum*3+3]
	footPosCalculate = legForwardKinematic(jointPosInput, legNum)
	print('inv:   ', legInverseKinematic(footPosCalculate, legNum).T)
	print('input: ', jointPosInput)
	footInfo= p.getLinkState(robot, 4*legNum+3)
	footPosWorld = np.array(footInfo[0]).reshape((3, 1))
	basePosWorld = np.array([xSet, ySet, zSet]).reshape((3, 1))
	rotMat = rotZ(yaw)@rotY(pitch)@rotX(roll)
	print(footPosWorld.T)
	print((basePosWorld+rotMat@footPosCalculate).T)
	p.stepSimulation()
	# time.sleep(dt)
