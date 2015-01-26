import math
import numpy as np

class Controller:

    def __init__(self, drone):
        self.drone = drone
        
        self.Kp_lin_vel = np.array([[5.0], [5.0], [5.0]])
        self.Kd_lin_vel = np.array([[2.5], [2.5], [0]])
        self.Kp_ang_vel = 10.0
        self.Kd_ang_vel = 5.0
        self.Kp_yaw_vel = 1.0

    def calculate_control_command(self,xdot_desired, yawdot_desired):
        
        world_acc_cmd = self.Kp_lin_vel * (xdot_desired - self.drone.xdot) - self.Kd_lin_vel * self.drone.xdoubledot;
        world_acc_cmd[2] = world_acc_cmd.item(2) + self.drone.g
        
        body_acc_cmd = np.dot(self.drone.rotation().transpose(), world_acc_cmd)
        body_angular_vel = self.drone.omega
        
        rates = np.array([
                          [self.Kp_ang_vel * (-body_acc_cmd.item(1) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(0)],
                          [self.Kp_ang_vel * (body_acc_cmd.item(0) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(1)],
                          [self.Kp_yaw_vel * (yawdot_desired - self.drone.thetadot.item(2))]
                          ]);
                          
        T_des = world_acc_cmd.item(2) / (math.cos(self.drone.theta.item(1)) * math.cos(self.drone.theta.item(0)))
        rates = np.vstack((rates, T_des))
        ctrl  = np.dot(self.drone.AinvKinvI, rates)
        
        return ctrl