'''
Created on 07.02.2014

@author: tatsch
'''
import math
import numpy as np


#Drone layout
#
#          1 O vr
#            |
#        x ^ |
#          | |
#            |          4
# O----------+----------O
# 2 vl   <--    |
#      y     |
#            |
#            |
#            O 3

class Drone():
    '''
    Model the drone
    '''

    L = 0.18  # distance between prop and COG in m
    m = 0.4365  # mass of the drone in kg, for hovering 5.8 A
    g = 9.81  # gm/s^2
    I = np.array([[0.007, 0, 0],
                  [0, 0.007, 0],
                  [0, 0, 0.012]])  # inertia matrix of the drone in gm3
    k_b = 0.1 # drag coefficient Nm/rpm2
    k_t =  0.73  # thrust coefficient in g/
    kd = 0.12  # air friction coefficent of the whole ardrone,

    x = np.array([[0.0],
                  [0.0],
                  [0.0]])
    xdot = np.zeros((3, 1))
    xdoubledot = np.zeros((3, 1))
    theta = np.zeros((3, 1))
    theta_body = np.zeros((3, 1))
    thetadot = np.zeros((3, 1))
    thetadoubledot = np.zeros((3, 1))
    omega = np.zeros((3, 1))


    def __init__(self):
        
        self.I_inv = np.linalg.inv(self.I)

        k = self.k_t
        kL = k * self.L
        b = self.k_b
        m = self.m
        Ixx = self.I.item((0, 0))
        Iyy = self.I.item((1, 1))
        Izz = self.I.item((2, 2))

        # K matrix is diagonal containing constants
        self.K = np.array([[kL, 0, 0, 0],
                           [0, kL, 0, 0],
                           [0,  0, b, 0],
                           [0,  0, 0, k]])
                           
        # A matrix is allocation matrix describing configuration of quadrotor
        self.A = np.array([[ 1, 1,-1,-1],
                           [ 1,-1,-1, 1],
                           [-1, 1,-1, 1],
                           [ 1, 1, 1, 1]])

        tmp = np.array([[Ixx, 0, 0, 0],
                        [0, Iyy, 0, 0],
                        [0, 0, Izz, 0],
                        [0, 0, 0, m  ]])

        self.KA = np.dot(self.K, self.A);
        
        #AinvKinvI Konverts desired angular rates and thrust to a control cmd
        self.AinvKinvI = np.dot(np.dot(np.linalg.inv(self.A), np.linalg.inv(self.K)), tmp)

        pass

    def angle_rotation_to_body(self):
        '''
        compute rotation matrix to convert angular velocities to body frame
        '''
        from math import sin, cos

        phi = self.theta.item(0);
        theta = self.theta.item(1);


        return np.array([[1, 0, -sin(theta)],
                      [0, -cos(phi), cos(theta) * sin(phi)],
                      [0, sin(phi), cos(theta) * cos(phi)]])

    def yaw_rotation(self):
        '''
        compute rotation matrix to convert angular velocities to body frame
        '''
        from math import sin, cos

        psi = self.theta.item(2);
        cpsi = cos(psi)
        spsi = sin(psi)
        return np.array([[cpsi, -spsi, 0],
                      [spsi, cpsi, 0],
                      [0, 0, 1]])

    def angle_rotation_to_world(self):
        '''
        compute rotation matrix to convert angular velocities to world frame
        '''
        from math import sin, cos, tan, fabs

        phi = self.theta.item(0);
        theta = self.theta.item(1);

        return np.array([[1, sin(phi) * tan(theta), cos(phi) * tan(theta)],
                      [0, cos(phi), -sin(phi)],
                      [0, sin(phi) / cos(theta), cos(phi) / cos(theta)]])

    def theta_in_body(self):
        return np.dot(self.angle_rotation_to_body(), self.theta)

    def thetadot_in_body(self):
        return np.dot(self.angle_rotation_to_body(), self.thetadot)

    def torques_thrust(self, inputs):
        return np.dot(self.KA, inputs)

    def rotation(self):  # translate angles to intertial/world frame
        phi = self.theta.item(0)
        theta = self.theta.item(1)
        psi = self.theta.item(2)

        c_phi = math.cos(phi);
        s_phi = math.sin(phi);
        c_theta = math.cos(theta);
        s_theta = math.sin(theta);
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)


        R = np.array([[c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
                      [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
                      [-s_theta, c_theta * s_phi, c_theta * c_phi]])

        return R
