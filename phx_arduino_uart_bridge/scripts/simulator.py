import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import time

class Simulator():

    start_time = 0  # in secs
    end_time   = 1
    dt         = 0.005


    def __init__(self, drone, controller):
        self.drone         = drone
        self.controller    = controller
        self.step_count    = 0
        self.x         = []
        self.y         = []
        self.z         = []
        self.roll      = []
        self.pitch     = []
        self.yaw       = []
        self.cmd1      = []
        self.cmd2      = []
        self.cmd3      = []
        self.cmd4      = []
        self.e_yaw     = []
        self.e_x       = []
        self.e_y       = []
        self.e_z       = []
        self.roll_des  = []
        self.pitch_des = []
        self.yaw_des   = []
        self.theta_desired    = np.array([[0.0], [0.0], [0.0]])
        self.thetadot_desired = np.array([[0.0], [0.0], [0.0]])
        self.x_desired        = np.array([[0.0], [0.0], [0.0]])
        self.xdot_desired     = np.array([[0.0], [0.0], [0.0]])
        self.drone.x          = np.array([[0.0],[0.0],[0.0]])
        self.drone.xdot       = np.array([[0.0],[0.0],[0.0]])
        self.drone.xdoubledot = np.array([[0.0],[0.0],[0.0]])
        self.drone.theta      = np.array([[0.0],[0.0],[0.0]])
        self.drone.omega      = np.array([[0.0],[0.0],[0.0]])
        self.drone.thetadot   = np.array([[0.0],[0.0],[0.0]])

    def get_drone_pose(self):
        return [self.drone.x.item(0), self.drone.x.item(1), self.drone.x.item(2), self.drone.theta.item(0), self.drone.theta.item(1), self.drone.theta.item(2)];

    def set_input(self, sim_input):

        self.xdot_desired[0]     = sim_input[0];
        self.xdot_desired[1]     = sim_input[1];
        self.thetadot_desired[2] = sim_input[2];
        self.xdot_desired[2]     = sim_input[3];
        self.xdot_desired        = np.dot(self.drone.yaw_rotation(), self.xdot_desired)

    def set_input_world(self, lin_vel, yaw_vel):
        self.xdot_desired[0]     = lin_vel[0]
        self.xdot_desired[1]     = lin_vel[1]
        self.xdot_desired[2]     = lin_vel[2]
        self.thetadot_desired[2] = yaw_vel;

    def simulate_step(self, dt):
        self.step_count += 1


        inputCurrents, acc = self.controller.calculate_control_command3(dt, self.xdot_desired, self.thetadot_desired.item(2))
        omega = self.drone.omega; # calculate current angular velocity in body frame

        torques_thrust       = self.drone.torques_thrust(inputCurrents)

        linear_acceleration  = self.linear_acceleration(torques_thrust[3], self.drone.theta, self.drone.xdot)  # calculate the resulting linear acceleration
        omegadot             = self.angular_acceleration(torques_thrust[0:3,0], omega)  # calculate resulting angular acceleration


        omega = omega + dt * omegadot # integrate up new angular velocity in the body frame

        self.drone.omega    = omega;
        self.drone.thetadot = np.dot(self.drone.angle_rotation_to_world().transpose(), omega)  # calculate roll, pitch, yaw velocities
        self.drone.theta    = self.drone.theta + dt * self.drone.thetadot  # calculate new roll, pitch, yaw angles


        self.drone.xdoubledot = linear_acceleration
        self.drone.xdot       = self.drone.xdot + dt * linear_acceleration  # calculate new linear drone speed
        self.drone.x          = self.drone.x + dt * self.drone.xdot  # calculate new drone position



    def deg2rad(self,degrees):
        return np.array(map(math.radians, degrees))

    def rotation(self, angles):  # translate angles to intertial/world frame
        phi   = angles.item(0)
        theta = angles.item(1)
        psi   = angles.item(2)

        c_phi   = math.cos(phi);
        s_phi   = math.sin(phi);
        c_theta = math.cos(theta);
        s_theta = math.sin(theta);
        c_psi   = math.cos(psi)
        s_psi   = math.sin(psi)

        # XYZ
        R = np.array([[c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
                      [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
                      [-s_theta, c_theta * s_phi, c_theta * c_phi]])
        return R

    def linear_acceleration(self, thrust, angles, xdot):
        gravity = np.array([[0], [0], [-self.drone.g]])
        R = self.rotation(angles)

        T      = np.dot(R, np.array([[0], [0], [thrust]]))
        F_drag = -self.drone.kd * xdot
        a      = gravity + (T + F_drag) / self.drone.m
        return a

    def angular_acceleration(self, torques, omega):
        # this transpose stuff really sucks
        omegaddot = np.dot(self.drone.I_inv, (torques.transpose() - np.cross(omega.transpose(), np.dot(self.drone.I, omega).transpose())).transpose());
        return omegaddot
