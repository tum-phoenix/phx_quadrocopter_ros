#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
# from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl


class Plotter():
    def __init__(self):
        rospy.init_node('plotter')
        self.sub = rospy.Subscriber('/phx/fc/altitude_hold', RemoteControl, self.altitudeCallback)

        self.i = 0
        self.heights = [0]
        self.graph_width = 100
        self.graph_height = 2;

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def altitudeCallback(self, altitude_hold_msg):
        print("Altitude Callback")
        # set previousAltitude to current Altitude in first call
        throttle = altitude_hold_msg.throttle
        self.heights.append(throttle)
        if(throttle > self.graph_height):
            self.graph_height *= 2

        if(self.heights.len() > self.graph_width):
            self.graph_width *= 2

        plt.plot(self.heights)
        # set height and width of graph
        plt.axis([0, self.graph_width, 0, self.graph_height])
        plt.show()


if __name__ == '__main__':
    try:
        plotter = Plotter()
        plotter.run()

    except rospy.ROSInterruptException:
        pass