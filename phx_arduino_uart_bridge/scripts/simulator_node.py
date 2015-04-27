#!/usr/bin/env python

import numpy as np
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Joy
import geometry_msgs.msg
from controller import Controller
from drone import Drone
from simulator import Simulator

seq = 0

def rcCallback(data):
    global seq
    throttle = data.axes[0]
    pitch = data.axes[1]
    roll = data.axes[2]
    yaw = data.axes[3]
    
    sim_input = [roll, pitch, yaw, 0.0]
    simulator.set_input(sim_input)
    simulator.simulate_step(0.005)
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.header.seq = seq
    t.child_frame_id = "drone"
    t.transform.translation.x = drone.x[0]
    t.transform.translation.y = drone.x[1]
    t.transform.translation.z = drone.x[2]
    q = tf.transformations.quaternion_from_euler(drone.theta[0], drone.theta[1], drone.theta[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)
    seq += 1


drone = Drone()
controller = Controller(drone)
simulator = Simulator(drone, controller)
rospy.init_node('simulator', anonymous=True)
rospy.Subscriber("/phoenix/stat_rc2", Joy, rcCallback)
br = tf2_ros.TransformBroadcaster()

if __name__ == '__main__':
    rospy.spin()
