from LineScannerPreview import PlaneRadialPlotter3D
from pyvis import Plotter3D, Axes3D, Grid3D, PlaneRadialPlotter3D
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu


class hokuyo3D:
    def __init__(self, test_parameter):
        print test_parameter

        self.current_pose = None
        # create first 3D window with a PlaneRadialPlotter3D
        self.radial_plot = PlaneRadialPlotter3D(start_angle=-2.08621382713, stop_angle=2.08621382713)
        self.radial_plot_ax = Axes3D(widget=self.radial_plot.widget)            # add a axis to the window of the PlaneRadialPlotter3D
        self.radial_plot_grid = Grid3D(widget=self.radial_plot.widget)          # add a grid to the window of the PlaneRadialPlotter3D
        # create second 3D window with a Plotter3D
        self.world_scatter = Plotter3D(app=self.radial_plot.app)                # here we have to specify the 3D app we are using. Only one app per runtime!!!
        self.axes = Axes3D(widget=self.world_scatter.widget)                    # again add a axis
        self.grid = Grid3D(widget=self.world_scatter.widget)                    # and a grid

        rospy.init_node('hokuyo_test')
        self.ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, self.callback_LaserScan)
        self.ros_subscribe_imu = rospy.Subscriber('/phx/imu', Imu, self.callback_imu)

        self.radial_plot.run()                                                  # to view all 3D windows we need to run the first 3D object we initialized

    def callback_LaserScan(self, new_LaserScan=LaserScan()):
        print len(new_LaserScan.ranges), new_LaserScan.angle_min, new_LaserScan.angle_max
        print self.current_pose
        data = np.array(new_LaserScan.ranges)[::-1]
        data[data > 6] = 6

        self.radial_plot.set_data(data=data)                                    # display input data in the radial_plot 3D window
        # add the new data points to the world_scatter
        #self.world_scatter.add_point(x, y, z)

    def callback_imu(self, new_imu=Imu()):
        #print new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z
        self.current_pose = [new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z]

H = hokuyo3D('hallo world')
