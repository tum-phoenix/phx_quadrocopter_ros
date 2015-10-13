import time

class LEDtab:
    def __init__(self, strip0, strip1, strip2, strip3, ros_publish_function=None):
        self.strip0 = strip0    # [ hSlider_color_r, hSlider_color_g, hSlider_color_b]
        self.strip1 = strip1
        self.strip2 = strip2
        self.strip3 = strip3

        self.ros_publish_function = ros_publish_function

    def send_all_strips(self):
        if self.ros_publish_function:
            self.ros_publish_function(0, self.strip0[0].value(), self.strip0[1].value(), self.strip0[2].value())
            time.sleep(0.005)
            self.ros_publish_function(1, self.strip1[0].value(), self.strip1[1].value(), self.strip1[2].value())
            time.sleep(0.005)
            self.ros_publish_function(2, self.strip2[0].value(), self.strip2[1].value(), self.strip2[2].value())
            time.sleep(0.005)
            self.ros_publish_function(3, self.strip3[0].value(), self.strip3[1].value(), self.strip3[2].value())