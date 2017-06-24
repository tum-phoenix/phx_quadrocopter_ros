import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


# republish topics for testing purposes:
# rosrun image_transport republish compressed in:=cam_front/image_raw out:=image

class PicturePublisherNode:
    def __init__(self):
        rospy.init_node('picture_publisher_controller')

        # self.publisher = rospy.Publisher('/cam_front/image_raw/compressed', CompressedImage, queue_size=1)
        self.publisher = rospy.Publisher('/bar_image/compressed', CompressedImage, queue_size=1)

        self.freq = 100  # Hz
        self.rate = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            # load image from file
            f = open('../qr_code_example.png', 'r')
            img = f.read()
            # convert image to ROS message
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = img

            self.publisher.publish(msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller_node = PicturePublisherNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
