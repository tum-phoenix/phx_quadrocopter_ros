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


def button_pressed(stuff='nothing'):
    print 'a button was pressed', stuff


# init osc ----------------------------------------------------------------------------------------------------------------------------------------------------
from OSC_com import *
osc_transmitter = OSCt(destination='192.168.0.41', port=10000)
osc_transmitter.send_connect()
osc_receiver = OSCr(osc_transmitter, port=10001)
osc_receiver.start()

# build osc bridge --------------------------------------------------------------------------------------------------------------------------------------------
# ros       >> osc  >> gui
# subscribe >> send >> receive
msg_structs = {'altitude': ['estimated_altitude', 'variation'],
               'status': ['cycletime', 'i2c_errors'],
               'battery': ['cell1', 'cell2', 'cell3', 'cell4'],
               'joy': ['axis0', 'axis1', 'axis2', 'axis3', 'aux0', 'aux1', 'aux2', 'aux3'],
               'motor': ['motor0', 'motor1', 'motor2', 'motor3'],
               'imu': ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'],
               'gps': ['latitude', 'longitude', 'altitude']}

input_messages = {'/phx/altitude_marvic': 'altitude',
                  '/phx/status_marvic': 'status',
                  '/phx/battery_marvic': 'battery',
                  '/phx/rc_marvic': 'joy',
                  '/phx/motor_marvic': 'motor',
                  '/phx/imu_marvic': 'imu',
                  '/phx/altitude_multiwii': 'altitude',
                  '/phx/status_multiwii': 'status',
                  '/phx/rc_multiwii': 'joy',
                  '/phx/motor_multiwii': 'motor',
                  '/phx/imu_multiwii': 'imu',
                  '/phx/gps_multiwii': 'gps'
                  }

for topic_title in input_messages.keys():
    osc_receiver.add_receive_message(topic_title)

# ros     << osc     << gui
# publish << receive << send
publishers = {'/phx/gui_rc': 'Joy',
              '/phx/gui_parameters': 'GUI_cmd'
              }

# link osc to gui ---------------------------------------------------------------------------------------------------------------------------------------------
# fill topic selector
selectable_topics = input_messages.keys()
ui_win.comboBox_topic_selection.addItems(selectable_topics)
ui_win.comboBox_topic_selection.setCurrentIndex(0)


def update_dropdowns(stuff='nothing'):
    print 'update_dropdowns:', stuff
    while ui_win.comboBox_sub_topic_selection.count() > 0:
        ui_win.comboBox_sub_topic_selection.removeItem(0)
    selected_topic = str(ui_win.comboBox_topic_selection.currentText())
    selected_topic_type = input_messages[selected_topic]
    if selected_topic_type in msg_structs.keys():
        selectable_sub_topics = msg_structs[selected_topic_type]
    else:
        selectable_sub_topics = []
    ui_win.comboBox_sub_topic_selection.addItems(selectable_sub_topics)
    ui_win.comboBox_sub_topic_selection.setCurrentIndex(0)

# fill sub_topic selector
update_dropdowns()

plot_subtopics = {}


def add_remove_selected_subtopic():
    selected_topic = str(ui_win.comboBox_topic_selection.currentText())
    selected_topic_type = input_messages[selected_topic]
    selected_sub_topic = str(ui_win.comboBox_sub_topic_selection.currentText())
    #selected_sub_topic_index = int(ui_win.comboBox_sub_topic_selection.CurrentIndex())
    full_selected_topic_path = selected_topic + '/' + selected_sub_topic
    if full_selected_topic_path not in plot_subtopics.keys():
        # add this subtopic to be plotted
        color = ['k', 'b', 'g', 'r', 'm', 'y'][len(plot_subtopics.keys())]
        index = msg_structs[selected_topic_type].index(selected_sub_topic)
        plot_subtopics[full_selected_topic_path] = [selected_topic, selected_sub_topic, index, color]
        print 'added', full_selected_topic_path, 'to list of plots'
    else:
        # remove this subtopic from the list of plots
        plot_subtopics.pop(full_selected_topic_path)
        print 'removed', full_selected_topic_path, 'from list of plots'


QtCore.QObject.connect(ui_win.button_add_remove_plot, QtCore.SIGNAL('clicked()'), add_remove_selected_subtopic)

QtCore.QObject.connect(ui_win.comboBox_topic_selection, QtCore.SIGNAL('currentIndexChanged(QString)'), update_dropdowns)




QtCore.QObject.connect(ui_win.button_send, QtCore.SIGNAL('clicked()'), button_pressed)


def send_gui():
    # gui stuff -----------------------------------------------------------------------------------------------------------------------------------------------
    rc_parameters = get_rc_parameters()
    set_rc_parameters_lcd(*rc_parameters)

    gui_parameters = get_gui_parameters()
    set_gui_parameters_lcd(*gui_parameters)

    # osc stuff -----------------------------------------------------------------------------------------------------------------------------------------------
    osc_transmitter.send_topic(topic='/phx/gui_parameters', payload=gui_parameters)
    osc_transmitter.send_topic(topic='/phx/gui_rc', payload=rc_parameters)


def mainloop():
    # data stuff ----------------------------------------------------------------------------------------------------------------------------------------------
    #global next_plot_update
    #if time.time() > next_plot_update:
    #    next_plot_update = time.time() + 0.25

    time_window = 1.0 * ui_win.horizontalSlider_delta_time.value() / 100.
    time_step = 1.0 - 1.0 * ui_win.horizontalSlider_time_point.value() / 100.
    print time_window, time_step

    plot_ax.cla()

    for to_plot_key in plot_subtopics.keys():
        selected_topic = plot_subtopics[to_plot_key][0]
        selected_subtopic = plot_subtopics[to_plot_key][1]
        selected_subtopic_index = plot_subtopics[to_plot_key][2]
        color = plot_subtopics[to_plot_key][3]
        data = osc_receiver.received_data[selected_topic]
        min = 0
        max = 1
        if len(data[0]) > 0:
            # only plot if there is data available!
            total_time = data[0][-1]
            last_sample_time = total_time * time_step
            biggest_window = 60
            min = last_sample_time - biggest_window * (1 - time_window)
            max = last_sample_time
            min_index = int(len(data[0]) * min/total_time)
            max_index = int(len(data[0]) * max/total_time)
            if min_index <= 0:
                min_index = 0
            if max_index >= len(data[0]):
                max_index = len(data[0])
            plot_ax.plot(data[0][min_index:max_index], data[1][selected_subtopic_index][min_index:max_index], color, label=selected_topic+'/'+selected_subtopic)
        plot_ax.set_xlim((min, max))

    plot_ax.legend()
    plot_canvas.draw()
    
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

send_timer = QtCore.QTimer()
send_timer.timeout.connect(send_gui)
send_interval_ms = 50
send_timer.start(send_interval_ms)



def quit_procedure():
    timer.stop()
    print 'stopping osc'
    osc_receiver.stop()
    osc_transmitter.stop()
    print 'end'


app.aboutToQuit.connect(quit_procedure)
app.exec_()

