__author__ = 'manuelviermetz'

import PyQt4
from PyQt4 import uic, QtCore, QtGui
from PyQt4.QtGui import QFileDialog
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt

# generate .py from .ui via pyuic4 gui_v0.ui -o gui_v0.py
from gui_v0 import Ui_MainWindow


class MainWindow(QtGui.QMainWindow):
    def __init__(self, *args, **kwargs):
        QtGui.QMainWindow.__init__(self, *args, **kwargs)
        self.keysPressed = []
        self.keyHits = []
        self.mouseClicks = []

    def mousePressEvent(self, ev):
        # print ev.button()
        # print self.view.ItemTransformOriginPointChange
        # print ev.x(), ev.y(), ev.pos()
        ev.accept()
        self.mouseClicks.append([ev.x(), ev.y(), ev.pos(), ev.button()])

    def keyPressEvent(self, ev):
        print ev.key(), QtCore.Qt.Key_Up
        ev.accept()
        if ev.key() not in self.keyHits:
            self.keyHits.append(ev.key())
        if ev.isAutoRepeat():
            return
        self.keysPressed.append(ev.key())

    def keyReleaseEvent(self, ev):
        # ev.accept()
        if ev.isAutoRepeat():
            return
        if ev.key() in self.keysPressed:
            self.keysPressed.remove(ev.key())
        else:
            print "key hit of", ev.key(), "not detected"

app = QtGui.QApplication([])
#win = PyQt4.uic.loadUi('gui_v0.ui')
##win = QtGui.QMainWindow()
win = MainWindow()
ui_win = Ui_MainWindow()
ui_win.setupUi(win)


win.setWindowTitle('phoenixGUI')
ui_win.statusbar.showMessage("starting up...")

next_plot_update = 0
plot_figure = plt.figure()
plot_canvas = FigureCanvas(plot_figure)
plot_ax = plot_figure.add_axes([0.10, 0.10, 0.85, 0.85])
plot_ax.set_xlabel('time')
ui_win.verticalLayout_plots.addWidget(plot_canvas)


def set_rc_parameters_lcd(axis0, axis1, axis2, axis3, button0, button1, button2, button3):
    ui_win.lcdNumber_rc_axis0.display(axis0)
    ui_win.lcdNumber_rc_axis1.display(axis1)
    ui_win.lcdNumber_rc_axis2.display(axis2)
    ui_win.lcdNumber_rc_axis3.display(axis3)
    ui_win.lcdNumber_rc_button0.display(button0)
    ui_win.lcdNumber_rc_button1.display(button1)
    ui_win.lcdNumber_rc_button2.display(button2)
    ui_win.lcdNumber_rc_button3.display(button3)


def set_rc_parameters_sliders(axis0, axis1, axis2, axis3, button0, button1, button2, button3):
    ui_win.verticalSlider_rc_axis0.setValue(axis0)
    ui_win.verticalSlider_rc_axis1.setValue(axis1)
    ui_win.verticalSlider_rc_axis2.setValue(axis2)
    ui_win.verticalSlider_rc_axis3.setValue(axis3)
    ui_win.verticalSlider_rc_button0.setValue(button0)
    ui_win.verticalSlider_rc_button1.setValue(button1)
    ui_win.verticalSlider_rc_button2.setValue(button2)
    ui_win.verticalSlider_rc_button3.setValue(button3)


def get_rc_parameters():
    ax0 = ui_win.verticalSlider_rc_axis0.value()
    ax1 = ui_win.verticalSlider_rc_axis1.value()
    ax2 = ui_win.verticalSlider_rc_axis2.value()
    ax3 = ui_win.verticalSlider_rc_axis3.value()
    but0 = ui_win.verticalSlider_rc_button0.value()
    but1 = ui_win.verticalSlider_rc_button1.value()
    but2 = ui_win.verticalSlider_rc_button2.value()
    but3 = ui_win.verticalSlider_rc_button3.value()
    return ax0, ax1, ax2, ax3, but0, but1, but2, but3


def get_gui_parameters():
    param0 = ui_win.horizontalSlider_parameter0.value()
    param1 = ui_win.horizontalSlider_parameter1.value()
    param2 = ui_win.horizontalSlider_parameter2.value()
    param3 = ui_win.horizontalSlider_parameter3.value()
    param4 = ui_win.horizontalSlider_parameter4.value()
    param5 = ui_win.horizontalSlider_parameter5.value()
    param6 = ui_win.horizontalSlider_parameter6.value()
    param7 = ui_win.horizontalSlider_parameter7.value()
    return param0, param1, param2, param3, param4, param5, param6, param7


def set_gui_parameters_slider(param0, param1, param2, param3, param4, param5, param6, param7):
    ui_win.horizontalSlider_parameter0.setValue(param0)
    ui_win.horizontalSlider_parameter1.setValue(param1)
    ui_win.horizontalSlider_parameter2.setValue(param2)
    ui_win.horizontalSlider_parameter3.setValue(param3)
    ui_win.horizontalSlider_parameter4.setValue(param4)
    ui_win.horizontalSlider_parameter5.setValue(param5)
    ui_win.horizontalSlider_parameter6.setValue(param6)
    ui_win.horizontalSlider_parameter7.setValue(param7)


def set_gui_parameters_lcd(param0, param1, param2, param3, param4, param5, param6, param7):
    ui_win.lcdNumber_parameter0.display(param0)
    ui_win.lcdNumber_parameter1.display(param1)
    ui_win.lcdNumber_parameter2.display(param2)
    ui_win.lcdNumber_parameter3.display(param3)
    ui_win.lcdNumber_parameter4.display(param4)
    ui_win.lcdNumber_parameter5.display(param5)
    ui_win.lcdNumber_parameter6.display(param6)
    ui_win.lcdNumber_parameter7.display(param7)


# init osc ----------------------------------------------------------------------------------------------------------------------------------------------------
from OSC_com import *
osc_transmitter = OSCt(destination='192.168.0.35', port=10000)
osc_transmitter.send_connect()
osc_receiver = OSCr(osc_transmitter, port=10001)
osc_receiver.start()

# build osc bridge --------------------------------------------------------------------------------------------------------------------------------------------
# ros       >> osc  >> gui
# subscribe >> send >> receive
input_messages = ['/phx/altitude_marvic',
                  '/phx/status_marvic',
                  '/phx/battery_marvic']
for code in input_messages:
    osc_receiver.add_receive_message(code)

# ros     << osc     << gui
# publish << receive << send
publishers = {'/phx/gui_rc': 'Joy',
              '/phx/gui_parameters': 'GUI_cmd'
              }

# link osc to gui ---------------------------------------------------------------------------------------------------------------------------------------------
ui_win.comboBox_topic_selection.addItems(input_messages)
ui_win.comboBox_topic_selection.setCurrentIndex(0)


def mainloop():
    # gui stuff -----------------------------------------------------------------------------------------------------------------------------------------------
    rc_parameters = get_rc_parameters()
    set_rc_parameters_lcd(*rc_parameters)

    gui_parameters = get_gui_parameters()
    set_gui_parameters_lcd(*gui_parameters)
    
    # data stuff ----------------------------------------------------------------------------------------------------------------------------------------------
    global next_plot_update
    if time.time() > next_plot_update:
        next_plot_update = time.time() + 0.1

        time_window =1.0 * ui_win.horizontalSlider_delta_time.value() / 100.
        time_step = 1.0 - 1.0 * ui_win.horizontalSlider_time_point.value() / 100.
        print time_window, time_step

        selected_topic = str(ui_win.comboBox_topic_selection.currentText())
        data = osc_receiver.received_data[selected_topic]
        plot_ax.cla()
        plots = len(data[1])

        try:
            total_time = data[0][-1]
        except:
            total_time = 1.0
        last_sample_time = total_time * time_step
        biggest_window = 60
        plot_ax.set_xlim((last_sample_time - biggest_window * (1 - time_window), last_sample_time))

        for i in range(0, plots):
            plot_ax.plot(data[0], data[1][i], 'b')
        plot_canvas.draw()

    osc_transmitter.send_topic(topic='/phx/gui_parameters', payload=gui_parameters)
    osc_transmitter.send_topic(topic='/phx/gui_rc', payload=rc_parameters)
    
    # osc stuff -----------------------------------------------------------------------------------------------------------------------------------------------
    print 'osc connection status: OSCr', osc_receiver.connection_status, 'OSCt', osc_transmitter.connection_status, osc_transmitter.sending_failed_counter, time.time() - osc_receiver.time_of_last_keep_alive_receiving
    osc_receiver.keep_alive()
    osc_transmitter.send_connect()

    print 'mainloop', win.keysPressed, get_rc_parameters()

win.show()
# QTimer
timer = QtCore.QTimer()
timer.timeout.connect(mainloop)
interval_ms = 50
timer.start(interval_ms)

app.exec_()

osc_receiver.stop()
osc_transmitter.stop()
print 'end'