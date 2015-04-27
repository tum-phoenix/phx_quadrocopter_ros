import math
import numpy as np

#when more order types are needed, a 'Order' superclass should be used

class RelativeOrder(object):
    def __init__(self, dx, dy, dz, dyaw):
        #relative movements
        self.dx = dx
        self.dy = dy
        self.dz = dz

        self.dyaw = (dyaw / 180.0) * math.pi

class MissionPlanner:
    def __init__(self):
        self.commands = []

    def forward(self, distance):
        return self._add_relative_command(distance, 0, 0, 0)

    def backward(self, distance):
        return self._add_relative_command(-distance, 0, 0, 0)

    def left(self, distance):
        return self._add_relative_command(0, distance, 0, 0)

    def right(self, distance):
        return self._add_relative_command(0, -distance, 0, 0)

    def up(self, distance):
        return self._add_relative_command(0, 0, distance, 0)

    def down(self, distance):
        return self._add_relative_command(0, 0, -distance, 0)

    def turn_left(self, angle):
        return self._add_relative_command(0, 0, 0, angle)

    def turn_right(self, angle):
        return self._add_relative_command(0, 0, 0, -angle)

    def _add_relative_command(self, dx, dy, dz, dyaw):
        self.commands.append(RelativeOrder(dx, dy, dz, dyaw))
        return self

    def add_commands(self, commands):
        #perform typecheck of appended commands
        for command in commands:
            if not isinstance(command, RelativeOrder):
                raise Exception("you can only add relative movement orders to the mission.")
        self.commands += commands


class PositionController:
    command_queue     = []
    command_queue_idx = -1

    setpoint_position = np.array([[0], [0], [0]])
    setpoint_yaw      = 0.0
    done              = False

    Kp_pos = 1.0
    Kd_pos = 1.0

    Kp_yaw = 2.0
    Kd_yaw = 4.0

    Limit_xy = 2.0
    Limit_z  = 0.5

    def __init__(self, drone, commands, do_log):
        self.drone         = drone
        self.command_queue = commands
        self.do_log        = do_log

    def distance_to_setpoint(self):
        pos_diff = self.setpoint_position - self.drone.x
        yaw_diff = 5 * (self.setpoint_yaw - self.drone.theta.item(2))

        return math.sqrt(np.dot(pos_diff.transpose(), pos_diff).item(0) + yaw_diff * yaw_diff)

    def update_setpoint(self, delta):
        world_delta            = np.dot(self.drone.yaw_rotation(), np.array([[delta.dx], [delta.dy], [delta.dz]]))
        self.setpoint_position = self.setpoint_position + world_delta
        self.setpoint_yaw     += delta.dyaw

    def clamp(self, value, limit):
        return max(min(value, limit), -limit)

    def compute_input(self):
        if self.done:
            return [0, 0, 0], 0;

        if self.command_queue_idx < 0 or self.distance_to_setpoint() < 0.05:
            self.command_queue_idx += 1

            if self.command_queue_idx < len(self.command_queue):
                self.update_setpoint(self.command_queue[self.command_queue_idx])
                if self.do_log:
                    print "updating setpoint, position:", self.setpoint_position.transpose(), "yaw:", self.setpoint_yaw
            else:
                self.done = True
                if self.do_log:
                    print "done"

        lin_vel_cmd = self.Kp_pos * (self.setpoint_position - self.drone.x) - self.Kd_pos * self.drone.xdot;
        yaw_vel_cmd = self.Kp_yaw * (self.setpoint_yaw - self.drone.theta.item(2)) - self.Kd_yaw * self.drone.thetadot.item(2)

        return [
            self.clamp(lin_vel_cmd.item(0), self.Limit_xy),
            self.clamp(lin_vel_cmd.item(1), self.Limit_xy),
            self.clamp(lin_vel_cmd.item(2), self.Limit_z)
        ], yaw_vel_cmd


class Controller():
    '''
    Implements a PIDController
    '''

    Kp_xy = 1
    Kd_xy = 1
    Ki_xy = 0

    Kp_roll = 3
    Kd_roll = 9
    Ki_roll = 0

    Kp_pitch = 3
    Kd_pitch = 9
    Ki_pitch = 0

    Kp_yaw = 1
    Kd_yaw = 1
    Ki_yaw = 0

    Kp_z = 1
    Kd_z = 1
    Ki_z = 0

    agressiveness_xy = 0.3
    agressiveness_z  = 0.3

    dt = 0.005

    def __init__(self, drone):
        self.drone         = drone
        self.errorIntegral = np.array([[0], [0], [0]])

        # TODO: tune gains
        self.Kp_angular_rate = np.array([[3.0], [3.0], [1.0]])
        self.Kp_attitude     = np.array([[5.0], [5.0], [1.0]])
        self.Kd_attitude     = np.array([[0.0], [0.0], [0.0]])
        self.Kp_zvelocity    = 5.0

        self.Kp_lin_vel = np.array([[5.0], [5.0], [5.0]])
        self.Kd_lin_vel = np.array([[2.5], [2.5], [0]])

        self.Kp_ang_vel = 10.0
        self.Kd_ang_vel = 5.0
        
        self.Kp_yaw_vel = 1.0


    def calculate_control_command3(self, dt, xdot_desired, yawdot_desired):

        world_acc_cmd    = self.Kp_lin_vel * (xdot_desired - self.drone.xdot) - self.Kd_lin_vel * self.drone.xdoubledot;
        world_acc_cmd[2] = world_acc_cmd.item(2) + self.drone.g
        body_acc_cmd     = np.dot(self.drone.rotation().transpose(), world_acc_cmd)
        body_angular_vel = self.drone.omega

        rates = np.array([
            [self.Kp_ang_vel * (-body_acc_cmd.item(1) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(0)],
            [self.Kp_ang_vel * (body_acc_cmd.item(0) / self.drone.g) - self.Kd_ang_vel * body_angular_vel.item(1)],
            [self.Kp_yaw_vel * (yawdot_desired - self.drone.thetadot.item(2))]
        ]);

        T_des = world_acc_cmd.item(2) / (math.cos(self.drone.theta.item(1)) * math.cos(self.drone.theta.item(0)))
        rates = np.vstack((rates, T_des))
        ctrl  = np.dot(self.drone.AinvKinvI, rates)
        return ctrl, world_acc_cmd
