import os
import argparse
import threading

from qt_gui.plugin import Plugin

from .gui_widget import GuiWidget

class Gui(Plugin):
    """
        Subclass of Plugin to provide visualization
        """
    def __init__(self, context):
        """
            :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
            """
        super(Gui, self).__init__(context)
        self.setObjectName('Gui')
        self._widget = GuiWidget(context)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    # Maybe this needs to run in a thread

    @staticmethod
    def shutdown_plugin(self):
        self._widget.shutdown_all()
