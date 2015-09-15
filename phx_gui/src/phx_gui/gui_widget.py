import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QIcon, QWidget, QMainWindow

class BagGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(BagGraphicsView, self).__init__()


class GuiWidget(QMainWindow): #QWidget
    """
        Widget for use with the OSC class
        Handles all widget callbacks
        """
    
    def __init__(self, context):
        """
            :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
            """
        super(GuiWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('phx_gui'), 'resource', 'gui_widget.ui')
        loadUi(ui_file, self, {'BagGraphicsView': BagGraphicsView})
        
        self.setObjectName('GuiWidget')