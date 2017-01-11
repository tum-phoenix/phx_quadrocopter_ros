import numpy as np
import rospy
import tf
import tf2_ros
from phx_uart_msp_bridge.msg import ControllerCmd


class TargetVector:
    def __init__(self):
        rospy.init_node('target_vector')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.shape=None
        self.transform=None
        self.target_diff = np.zeros(3)
        self.target_rot = np.zeros(3)
        self.r = rospy.Rate(10)
        self.shape_sub = rospy.Subscriber('/phx/controller_commands', ControllerCmd, self.shapeCallback)


    def shapeCallback(self,ControllerCmd):
        self.shape=ControllerCmd.shape


    def get_target_vector(self,shape):
        if shape=='Heart':
            try:
                self.transform = self.tfBuffer.lookup_transform('Heart', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape=='Star':
            try:
                self.transform = self.tfBuffer.lookup_transform('Star', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape=='Square':
            try:
                self.transform = self.tfBuffer.lookup_transform('Square', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape=='Pentagon':
            try:
                self.transform = self.tfBuffer.lookup_transform('Pentagon', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape=='Circle':
            try:
                self.transform = self.tfBuffer.lookup_transform('Circle', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape=='Triangle':
            try:
                self.transform = self.tfBuffer.lookup_transform('Triangle', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        elif shape is None:
            print 'no shape'

        if self.transform is not None:
            self.target_diff[0] = self.transform.transform.translation.x
            self.target_diff[1] = self.transform.transform.translation.y
            self.target_diff[2] = self.transform.transform.translation.z
            quaternion = (self.transform.transform.rotation.x,
                          self.transform.transform.rotation.y,
                          self.transform.transform.rotation.z,
                          self.transform.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.target_rot[0] = euler[0]   # pitch
            self.target_rot[1] = euler[1]   # roll
            self.target_rot[2] = euler[2]   # yaw
            print self.target_diff
            print self.target_rot
        else:
            print 'no transform'

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == '__main__':
    try:
        controller_node = TargetVector()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
