import pyqtgraph
import numpy as np
from PyQt4 import QtGui
import matplotlib.pyplot as plt
from goompy import GooMPy


class VIDEOtab:
    def __init__(self, graphicsView_video, video_active=lambda: False, video_reset=lambda: False):
        self.live_image = np.zeros((480, 640), dtype=np.uint8)
        self.time_of_last_image = 0
        self.video_fps = 0
        self.image_mask = np.zeros((480, 640), dtype=np.uint8)

        self.video_item = pyqtgraph.ImageView(graphicsView_video)

        self.video_auto_range = False
        self.video_auto_level = False
        self.video_auto_histogram_range = False
        self.video_item.setFixedWidth(611)
        self.video_item.setFixedHeight(421)
        self.video_item.setImage(np.swapaxes(self.live_image, 0, 1), levels=(0, 255), autoHistogramRange=False)

        self.video_active = video_active
        self.video_reset = video_reset

    def update_video_click(self):
        while len(self.video_item.mouse_clicks) > 0:
            click = self.video_item.mouse_clicks.pop()
            x0, x1, button = click[1], click[2], click[4]
            if button == 1:
                borders = self.video_item.imageItem.sceneBoundingRect()
                if x0 < borders.left() or x0 > borders.right() or x1 < borders.top() or x1 > borders.bottom():
                    # click outside the image
                    #print 'new click outside'
                    break
                img_x0 = (x0 - borders.left()) / borders.width()
                img_x1 = (x1 - borders.top()) / borders.height()
                y = int(img_x0 * self.image_mask.shape[1])
                x = int(img_x1 * self.image_mask.shape[0])

                #print 'new click inside', x, y, button
                self.image_mask *= 0
                for dx in range(-10, 12):
                    self.image_mask[x + dx, y] = 100
                    self.image_mask[x + dx, y + 1] = 100
                for dy in range(-10, 12):
                    self.image_mask[x, y + dy] = 100
                    self.image_mask[x + 1, y + dy] = 100

    def update_video(self):
        if self.video_active():
            if self.video_reset():
                self.video_item.setImage(np.swapaxes(self.live_image + self.image_mask, 0, 1), levels=(0, 255), autoHistogramRange=False)
            else:
                self.video_item.setImage(np.swapaxes(self.live_image + self.image_mask, 0, 1), autoLevels=self.video_auto_range, autoRange=self.video_auto_range, autoHistogramRange=self.video_auto_histogram_range)
            self.video_item.update()