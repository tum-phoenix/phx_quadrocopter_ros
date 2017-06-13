import rospy


class ZbarReaderNode:
    def __init__(self):
        rospy.init_node('zbar_reader_controller')
        self.sub_imu = rospy.Subscriber('/barcode', None, self.print_barcode)

        self.freq = 100  # Hz
        self.rate = rospy.Rate(self.freq)

    def print_barcode(self, barcode_msg):
        print barcode_msg

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller_node = ZbarReaderNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass