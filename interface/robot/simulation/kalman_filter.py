import numpy as np
import pandas as pd
from math import *
import matplotlib.pyplot as plt

def rotX(roll):
    res = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    return res

def rotY(pitch):
    res = np.array([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    return res

def rotZ(yaw):
    res = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    return res

def cross_mat(v):
    v = v.reshape(3)
    res = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
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


def getLegJacobian(angle, leg):
    x, y, k = 0.09735, -0.2, -0.21012
    if leg == 1 or leg == 3:
        x = -x
    q1, q2, q3 = angle[0], angle[1], angle[2]
    c1, s1 = cos(q1), sin(q1)
    c2, s2 = cos(q2), sin(q2)
    c3, s3 = cos(q3), sin(q3) 
    J = np.array([[0, -k*((c2)*(c3)-(s2)*(s3))-y*(c2), -k*((c2)*(c3)-(s2)*(s3))],
                  [k*((c1)*(c2)*(c3)-(c1)*(s2)*(s3))-x*(s1)+y*(c1)*(c2), -k*((c2)*(s1)*(s3)+(c3)*(s1)*(s2))-y*(s1)*(s2), -k*((c2)*(s1)*(s3)+(c3)*(s1)*(s2))],
                  [k*((s1)*(s2)*(s3)-(c2)*(c3)*(s1))-x*(c1)-y*(c2)*(s1), -k*((c1)*(c2)*(s3)+(c1)*(c3)*(s2))-y*(c1)*(s2), -k*((c1)*(c2)*(s3)+(c1)*(c3)*(s2))]])
    return J


class DataRead():
    def __init__(self, file_path):
        data = pd.read_csv(file_path, header=1, encoding='utf-8')
        self.data_mat = data.iloc[:, :].values
        self.time = self.data_mat[:, 2] - self.data_mat[0, 2]
        self.joint_pos = self.data_mat[:, 3:15]
        self.joint_vel = self.data_mat[:, 15:27]
        self.joint_tau = self.data_mat[:, 27:39]
        self.rpy = self.data_mat[:, 63:66]
        self.acc = self.data_mat[:, 66:69]
        self.omg = self.data_mat[:, 69:72]
        self.state = self.data_mat[:, 72]
        self.data_length = len(self.time)

class SimpleKalmanFilter():
    def __init__(self, dt=0.001) -> None:
        self.dt = dt
        eye3 = np.identity(3)

        self.A = np.zeros((6, 6))
        self.A[0:3, 0:3] = eye3
        self.A[0:3, 3:6] = self.dt * eye3
        self.A[3:6, 3:6] = eye3

        self.B = np.zeros((6, 3))
        self.B[3:6, :] = 0.5 * self.dt * self.dt * eye3
        self.B[3:6, :] = self.dt * eye3

        self.H = np.zeros((4, 6))
        self.H[:, 2::] = np.identity(4)

        self.P = np.identity(6)
        self.Z = np.zeros((4, 1))

        self.Q = np.zeros((6, 6))
        self.Q[0:3, 0:3] = 0.00002 * eye3
        self.Q[3:6, 3:6] = 0.0002 * eye3

        self.R = np.zeros((4, 4))
        self.R = np.diag([0.02, 7.5, 7.5, 7.5])
        # print('R: ', self.R)

        self.velocity_record = np.zeros((3, 1))
        self.height_record = 0
        self.off_land_cnt = 0
        self.X = np.zeros((6, 1))

        self.pos_world = np.zeros((3, 1))
        self.vel_world = np.zeros((3, 1))
        self.vel_body = np.zeros((3, 1))
        self.contact_sum = 0
        

    def run(self, rBody, aBody, xyz_pos, xyz_vel, contact):
        g = np.array([[0], [0], [-9.81]])
        a = rBody @ aBody + g
        contact_sum = np.sum(contact)
        # print('sum:  ', contact_sum)
        if contact_sum != 0:
            pos_world = np.zeros((3, 1))
            vel_world = np.zeros((3, 1))
            for i in range(4):
                pos_world = pos_world + contact[i, 0] * np.mat(xyz_pos[i, :]).T
                vel_world = vel_world + contact[i, 0] * np.mat(xyz_vel[i, :]).T
            pos_world = pos_world / contact_sum
            vel_world = vel_world / contact_sum     
            self.Z = np.vstack((pos_world[2, 0], vel_world))
            # print("calc:", pos_world, vel_world, self.Z)
            
        else:
            self.off_land_cnt += 1
            off_land_time = self.off_land_cnt * self.dt
            vel_world = self.velocity_record + off_land_time * g
            height = self.height_record + off_land_time * self.velocity_record[2, 0] + 0.5 * off_land_time * off_land_time * g[2, 0]
            self.Z = np.vstack((height, vel_world))
            # print('offland: ', self.Z, height, vel_world)

        self.X = self.A @ self.X + self.B @ a
        self.P = self.A @ self.P @ self.A.T + self.Q
        Kg = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        # print('kg: ', Kg)
        self.X = self.X + Kg @ (self.Z - self.H @ self.X)
        self.P = (np.identity(6) - Kg @ self.H) @ self.P

        self.pos_world = self.X[0:3, :]
        self.vel_world = self.X[3::, :]
        self.vel_body = rBody.T @ self.vel_world
        self.contact_sum = contact_sum

        if contact_sum != 0:
            self.off_land_cnt = 0
            self.velocity_record = self.X[3::, :]
            self.height_record = self.X[2, 0]
            # print('height_record: ', self.height_record)
            # print('velocity_record: ', self.velocity_record)
        

    def init(self, height):
        self.X = np.zeros((6, 1))
        self.X[2] = height
        self.velocity_record = np.zeros((3, 1))
        self.height_record = 0
        self.off_land_cnt = 0       



if __name__== '__main__':
    skf = SimpleKalmanFilter()
    file_path = "/home/ysc/ysc_code/deeprobotics_rl_deploy/data/20241026_142527.csv"
    dr = DataRead(file_path)
    
    x_odom = []
    y_odom = []
    z_odom = []
    for i in range(dr.data_length):
        if i!=0 and dr.state[i]==6 and dr.state[i-1]!=6:
            skf.init(0.35)
        if dr.state[i]!=6:
            continue
        joint_pos = np.array(dr.joint_pos[i, :]).reshape(12, 1)
        joint_vel = np.array(dr.joint_vel[i, :]).reshape(12, 1)
        joint_tau = np.array(dr.joint_tau[i, :]).reshape(12, 1)
        rot_mat = rotZ(dr.rpy[i, 2])@rotY(dr.rpy[i, 1])@rotX(dr.rpy[i, 0])
        acc = np.array(dr.acc[i, :]).reshape((3, 1))
        omega_body = np.array(dr.omg[i, :]).reshape((3, 1))
        leg_pos = np.zeros((4, 3))
        leg_vel = np.zeros((4, 3))
        contact = np.zeros((4, 1))
        for j in range(4):
            J = getLegJacobian(joint_pos[3*j:3*j+3, 0], j)
            vel_local = (J@joint_vel[3*j:3*j+3, 0]).reshape(3, 1)
            pos_local = legForwardKinematic(joint_pos[3*j:3*j+3, 0], j)
            leg_pos[j, :] = (rot_mat@pos_local).T
            # print(vel_local)
            # print((cross_mat(rot_mat@omega_body)@pos_local+rot_mat@vel_local).T)
            leg_vel[j, :] = (cross_mat(rot_mat@omega_body)@pos_local+rot_mat@vel_local).T
            esti_force = (rot_mat@np.linalg.inv(J.T))@(-joint_tau[3*j:3*j+3, 0].T)
            if esti_force[2] > 30:
                contact[j, 0] = 1
        skf.run(rot_mat, acc, leg_pos, leg_vel, contact)
        x_odom.append(skf.X[0, 0])
        y_odom.append(skf.X[1, 0])
        z_odom.append(skf.X[2, 0])


    fig1, axes = plt.subplots(nrows=3, ncols=1, sharex=True, num='XYZ')
    fig1.suptitle('Robot Odometery', family='Times New Roman', fontsize=25)
    title_name = r'$\it{X}$'
    axe = axes[0]
    axe.plot(x_odom, label=r'$\it{x_{data}}$', color='r')
    # axe.plot(data.time, lkf_cal_pos_rec[:, 0], label=r'$\it{x_{lkf}}$', color='g')
    # axe.plot(data.time, skf_cal_pos_rec[:, 0], label=r'$\it{x_{skf}}$', color='b')
    axe.legend(loc="upper right")
    axe.set_title(title_name, family='Times New Roman', fontsize=15)
    axe.set_ylabel('x/m', family='Times New Roman', fontsize=18)
    axe.grid()
    title_name = r'$\it{Y}$'
    axe = axes[1]
    axe.plot(y_odom, label=r'$\it{y_{data np.ptp(z_odom)}}$', color='r')
    # axe.plot(data.time, lkf_cal_pos_rec[:, 1], label=r'$\it{y_{lkf}}$', color='g')
    # axe.plot(data.time, skf_cal_pos_rec[:, 1], label=r'$\it{y_{skf}}$', color='b')
    axe.legend(loc="upper right")
    axe.set_title(title_name, family='Times New Roman', fontsize=15)
    axe.set_ylabel('y/m', family='Times New Roman', fontsize=18)
    axe.grid()
    title_name = r'$\it{Z}$'
    axe = axes[2]
    axe.plot(z_odom, label=r'$\it{z_{data}}$', color='r')
    # axe.plot(data.time, lkf_cal_pos_rec[:, 2], label=r'$\it{z_{lkf}}$', color='g')
    # axe.plot(data.time, skf_cal_pos_rec[:, 2], label=r'$\it{z_{skf}}$', color='b')
    axe.legend(loc="upper right")
    axe.set_title(title_name, family='Times New Roman', fontsize=15)
    axe.set_ylabel('z/m', family='Times New Roman', fontsize=18)
    axe.set_xlabel('time/s', family='Times New Roman', fontsize=18)
    axe.grid()  

    fig3 = plt.figure(num='3D')
    title_name = r'$\it{XYZ}$'
    ax = fig3.add_subplot(111)
    # ax.set_box_aspect((np.ptp(x_odom), np.ptp(y_odom)))#numpy ptp 数组中的最大值-最小值
    # ax.set_aspect('auto', adjustable='box')
    ax.plot(x_odom, y_odom, label=r'$\it{lkf}$', color='g') 
    ax.legend(loc="upper right")
    plt.show()


