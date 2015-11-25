from LineScannerPreview import PlaneRadialPlotter3D
from pyvis import Plotter3D, Axes3D, Grid3D
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

import geometry_msgs.msg


class hokuyo3D:
    def __init__(self, test):
        print test
        self.current_pose = None
        #self.plot_output = PlaneRadialPlotter3D(data_shape=681, start_angle=-2.08621382713, stop_angle=2.08621382713)

        self.axes = Axes3D()
        self.grid = Grid3D(widget=self.axes.widget)
        self.scatter = Plotter3D(widget=self.axes.widget)

        rospy.init_node('hokuyo_test')
        self.ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, self.callback_LaserScan)
        self.ros_subscribe_imu = rospy.Subscriber('/phx/imu', Imu, self.callback_imu)

        #self.plot_output.run()
        self.axes.run()

    def callback_LaserScan(self, new_LaserScan=LaserScan()):
        print len(new_LaserScan.ranges), new_LaserScan.angle_min, new_LaserScan.angle_max
        print self.current_pose
        data = np.array(new_LaserScan.ranges)[::-1]
        data[data > 6] = 6

        self.scatter.add_point(x=, y, z)
        #self.plot_output.set_data(data=data)

    def callback_imu(self, new_imu=Imu()):
        #print new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z
        self.current_pose = [new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z]

H = hokuyo3D('hallo world')
