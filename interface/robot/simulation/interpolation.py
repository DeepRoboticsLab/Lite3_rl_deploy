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
import time
import matplotlib.pyplot as plt

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

def getCubicSplinePosVel(x0, v0, xf, vf, t, T):
	if t > T:
		return xf, vf
	d=x0
	c=v0
	a=(vf*T-2*xf+v0*T+2*x0)/T**3
	b=(3*xf-vf*T-2*v0-3*x0)/T**2
	return a*t**3+b*t**2+c*t+d, 3*a*t**2+2*b*t+c


def getLegSwingTrajectoryPos(p0, v0, pf, vf, runTime, period=1.0, swingHeight=0.08):
	if runTime < 0:
		return p0, v0
	px, vx = getCubicSplinePosVel(p0[0, 0], v0[0, 0], pf[0, 0], vf[0, 0], runTime, period)
	py, vy = getCubicSplinePosVel(p0[1, 0], v0[1, 0], pf[1, 0], vf[1, 0], runTime, period)
	if runTime < period/2.:
		pz, vz = getCubicSplinePosVel(p0[2, 0], v0[2, 0], p0[2, 0]+swingHeight, 0, runTime, period/2.)
	else:
		pz, vz = getCubicSplinePosVel(p0[2, 0]+swingHeight, 0, pf[2, 0], vf[2, 0], runTime-period/2., period/2.)
	return np.array([[px], [py], [pz]]), np.array([[vx], [vy], [vz]])
		
	
	
	
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
		p.resetJointState(robot, j, default)
		# jointPos.append(p.addUserDebugParameter(' '+jointName, lower, upper, default))
	print(j, " : ", jointName, " qIndex : ", jointInfo[3], " uIndex : ", jointInfo[4], "lower : ", lower, 'upper : ', upper)

dt = 1./1000.
runCnt=0
jointForceLimit = 200

numOfRevolute = len(jointIndexList)
p.setTimeStep(dt)
flInitPos = np.array([-0.15+0.1745, 0.02+0.062+0.09735, -0.35]).reshape((3, 1))
flInitVel = np.zeros((3, 1))
flFinalPos = np.array([0.10+0.1745, 0.00+0.062+0.09735, -0.35]).reshape((3, 1))
flJointAngle = legInverseKinematic(flInitPos, 0)
quat = p.getQuaternionFromEuler([0, 0, 0])
p.resetBasePositionAndOrientation(robot, [0, 0, 0.5], quat)
for i in range(3):
	p.resetJointState(robot, jointIndexList[i], flJointAngle[i, 0])
startTime=1.
swingPeriod = 1.
runTime = 0


runTimeList = []
flFootPosList = []
flJointAngleList = []
# 进行仿真
while runTime < startTime+swingPeriod+1.:
	runCnt+=1
	runTime=runCnt*0.001
	if runTime >= startTime:
		flFootPos, _ = getLegSwingTrajectoryPos(flInitPos, np.zeros((3, 1)), flFinalPos, np.zeros((3, 1)), runTime-startTime, swingPeriod)
		flJointAngle = legInverseKinematic(flFootPos, 0)
		if runTime - startTime <= swingPeriod:
			runTimeList.append(runTime)
			flFootPosList.append(flFootPos.reshape(3).tolist())
			flJointAngleList.append(flJointAngle.reshape(3).tolist())
	for i in range(3):
		p.resetJointState(robot, jointIndexList[i], flJointAngle[i, 0])

	p.stepSimulation()
	time.sleep(dt)

# print(flJointAngleList)
fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)
fig.suptitle('FL Foot Position')
titleName = ['X', 'Y', 'Z']
for i in range(3):
	axes[i].plot(runTimeList, [row[i] for row in flFootPosList])
	axes[i].set_title(titleName[i])
	axes[i].grid()
axes[-1].set_xlabel('time/s')

fig, axes = plt.subplots(nrows=3, ncols=1, sharex=True)
fig.suptitle('FL Joint Angle')
titleName = ['HipX', 'HipY', 'Knee']
for i in range(3):
	axes[i].plot(runTimeList, [row[i] for row in flJointAngleList])
	axes[i].set_title(titleName[i])
	axes[i].grid()
axes[-1].set_xlabel('time/s')
plt.show()
