#!/usr/bin/env python
import numpy as np          # mathematics
import math
import time

import rospy                # ROS interface
import tf2_ros              # ROS transformation support
import tf

# ROS message types used throughout this script
import geometry_msgs.msg
import sensor_msgs.msg

import numpy as np
import numpy.linalg as la


def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)


class naive_gps_controller:
    def __init__(self):
        self.target_pos = np.zeros(3)
        self.copter_pos = np.zeros(3)
        self.copter_rot = np.zeros(3)
        self.cost_ranges = None

        # initial definition of globally used variables
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # listening for Attitude msg on /phx/fc/attitude topic
        self.ros_subscribe_target_position = rospy.Subscriber('/clicked_point',
                                                              geometry_msgs.msg.PointStamped,
                                                              self.receive_new_target)
        self.ros_subscribe_cost_map = rospy.Subscriber('/cost_ranges',
                                                       sensor_msgs.msg.LaserScan,
                                                       self.receive_new_cost_scan)

        # calc speed
        self.prev_position = np.zeros(3)
        self.prev_time_stamp = time.time()
        self.prev_velocities = [0, 0, 0, 0, 0]

        # controller variables
        self.p = 1
        self.i = 1
        self.sum_i_x = 0
        self.sum_i_y = 0
        self.i_cutoff = 10

    def receive_new_target(self, input_point):
        # print 'new point:', input_point
        self.target_pos[0] = input_point.point.x
        self.target_pos[1] = input_point.point.y
        self.target_pos[2] = input_point.point.z

    def receive_new_cost_scan(self, input_scan):
        # print 'new ranges'
        self.cost_ranges = input_scan

    def get_cur_pos(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'footprint', rospy.Time())
            self.copter_pos[0] = trans.transform.translation.x
            self.copter_pos[1] = trans.transform.translation.y
            self.copter_pos[2] = trans.transform.translation.z
            quaternion = (trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.copter_rot[0] = euler[0]   # pitch
            self.copter_rot[1] = euler[1]   # roll
            self.copter_rot[2] = euler[2]   # yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if self.copter_pos is not self.prev_position:
            distance = np.sqrt( np.sum( np.power(self.copter_pos - self.prev_position, 2)))
            print 'distance', distance
            dt = time.time() - self.prev_time_stamp
            vel = distance / dt
            self.prev_time_stamp = time.time()
            self.prev_position = np.copy(self.copter_pos)
            self.prev_velocities = self.prev_velocities[1:]
            self.prev_velocities.append(vel)
            print 'velocity', np.mean(self.prev_velocities)

        return self.copter_pos

    def calc_directions(self):
        # dx = self.target_pos[0] - self.copter_pos[0]
        # dy = self.copter_pos[0] - self.copter_pos[1]
        # dx = 2
        # dy = 1
        # vector_map = np.array([dx, dy])
        #
        # magnetometer = - np.pi * 0/180
        # angle = magnetometer + np.pi * 90/180 # self.copter_rot[2]
        # vector_copter = np.array([np.cos(angle), np.sin(angle)])
        # rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
        #                       [np.sin(angle), np.cos(angle)]])
        # vector_copter = np.dot(rot_matrix, vector_map)

        self.target_pos = np.array([-2., 1., 0.])
        #self.copter_pos = np.array([0., 0., 0.])

        self.sum_i_x += self.target_pos[0] - self.copter_pos[0]
        self.sum_i_y += self.target_pos[1] - self.copter_pos[1]
        if abs(self.sum_i_x) >= self.i_cutoff:
            self.sum_i_x = self.sum_i_x / abs(self.sum_i_x) * self.i_cutoff
        if abs(self.sum_i_y) >= self.i_cutoff:
            self.sum_i_y = self.sum_i_y / abs(self.sum_i_y) * self.i_cutoff

        controlCommand_p_x = (self.target_pos[0] - self.copter_pos[0]) * self.p
        controlCommand_i_x = self.sum_i_x * self.i
        # controlCommand_d_x = (set_d - (self.copter_pos[0] - self.previousAltitude)) * self.d

        controlCommand_p_y = (self.target_pos[1] - self.copter_pos[1]) * self.p
        controlCommand_i_y = self.sum_i_y * self.i
        # controlCommand_d_y = (set_d - (self.copter_pos[1] - self.previousAltitude)) * self.d

        controlCommand_x = controlCommand_p_x + controlCommand_i_x
        controlCommand_y = controlCommand_p_y + controlCommand_i_y
        print 'controlCommand_x', controlCommand_p_x, controlCommand_i_x, '\tcontrolCommand_y', controlCommand_p_y, controlCommand_i_y

# initialize node|
rospy.init_node('position_hold_node')

# initialize 'speed'-limit for endless loop
r = rospy.Rate(10)

controller = naive_gps_controller()


# start endless loop until rospy.is_shutdown()
while not rospy.is_shutdown():
    controller.get_cur_pos()
    print 'current position:', controller.copter_pos
    print 'current rotation:', controller.copter_rot
    print 'current target:', controller.target_pos

    controller.calc_directions()

    r.sleep()       # this prevents the node from using 100% CPU
