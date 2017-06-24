#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Joy


class RCReflectNode:
    """
    This node's function is simply to read the original 'human' RC input and send it to the flight controller as an active command.
    Therefore this node has no scientific relevance, but is a nice example and can be used for a proof of concept.
    """
    def __init__(self):
        """
        initiate ros node, init subscribers and publishers, and setup the necessary variables
        :return: None
        """
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]        # setup list which should in future hold active rc parameters

        rospy.init_node('RC_reflect_node')                                      # init ros node
        self.sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rcCallback)     # init ros subscriber on /phx/rc_marvic message channel and link self.rcCallback
        self.pub = rospy.Publisher('/phx/rc_computer', Joy)                     # init ros publisher of computer rc which is transmitted to the fc

        # init some kind of ugly main loop ... not too nice ... but it works pretty well.
        self.freq = 100                             # define main loop frequency in Hz
        r = rospy.Rate(self.freq)                   # init a rospy.Rate() object doing the timing in future
        self.counter_input = 0                      # setup some more useful variables
        self.counter_output = 0                     # setup some more useful variables
        self.start_time = time.time()               # setup some more useful variables

        while not rospy.is_shutdown():              # run the mainloop until ros is shut down
            # main loop stuff goes here:
                                                    # this node has nothing to do in the mainloop

            r.sleep()                               # let the rospy.Rate object controll the rate depending on the set frequency

    def rcCallback(self, joy_msg):
        """
        This function is meant to receive an incoming joy message and analyse the info.
        In this case every second incoming message is used to update the self.input_rc list of current 'human' remote data and then published to the flight controller.
        :param joy_msg:
        :return:
        """
        self.counter_input += 1

        if self.counter_input % 2 == 0:
            self.counter_output += 1
            joy_msg = Joy()

            # Replay and override current rc
            joy_msg.axes = self.input_rc[0:4]
            joy_msg.buttons = self.input_rc[4:]
            self.input_rc[0] = joy_msg.axes[0]
            self.input_rc[1] = joy_msg.axes[1]
            self.input_rc[2] = joy_msg.axes[2]
            self.input_rc[3] = joy_msg.axes[3]
            self.input_rc[4] = joy_msg.buttons[0]
            self.input_rc[5] = joy_msg.buttons[1]
            self.input_rc[6] = joy_msg.buttons[2]
            self.input_rc[7] = joy_msg.buttons[3]
            self.pub.publish(joy_msg)

        if self.counter_input % 100 == 0:
            print 'rc_tunnel_node.py >>', self.counter_input/(time.time()-self.start_time), 'rc inputs/sec  ', self.counter_output/(time.time()-self.start_time), 'rc outputs/sec'


# if this script is executed directly the following if statement is true and the node is started!
if __name__ == '__main__':
    controller_node = RCReflectNode()
