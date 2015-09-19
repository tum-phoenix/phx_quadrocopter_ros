__author__ = 'manuelviermetz'

from PyQt4 import uic, QtCore, QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import time

# import ROS
import rospy
from phx_arduino_uart_bridge.msg import Servo
from sensor_msgs.msg import NavSatFix


# generate .py from .ui via pyuic4 gui_v0.ui -o gui_v0.py
from gui_v1 import Ui_MainWindow
import pyqtgraph as pg


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


win.setWindowTitle('gauge GUI - made for ROS')
ui_win.statusbar.showMessage("starting up...")

##########################################################################################
# init tabs left
##########################################################################################
# text output
ui_win.textBrowser.setText('test text')

# gps tab
gps_data = [[], []]  # [[lon], [lat]]
gps_positions = {}

ui_win.gps_graphicsView.plotItem.showGrid(x=True, y=True, alpha=0.2)
gps_qtgraph_plot = ui_win.gps_graphicsView.plotItem.plot()
# gps_qtgraph_plot.setData([1, 2, 3], [1, 3, 1])

gps_scatter_plot = pg.ScatterPlotItem()
gps_scatter_plot.setData(gps_positions.values())
# gps_scatter_plot.setData([{'pos': (11, 42), 'symbol': 'o'}, ...])
ui_win.gps_graphicsView.addItem(gps_scatter_plot)

# label = pg.TextItem(text='test')
# ui_win.gps_graphicsView.addItem(label)
# label.setPos(11, 42)
# ui_win.gps_graphicsView.removeItem(label)
gps_position_labels = {}


def update_gps_plot(path=True, points=True):
    if path:
        # update gps path
        gps_qtgraph_plot.setData(gps_data[0], gps_data[1])
    if points:
        # update gps points
        gps_scatter_plot.setData(gps_positions.values())
        # add labels and update position
        for label in gps_positions.keys():
            if label not in gps_position_labels.keys():
                if label == 'home':
                    color = (255, 255, 0)
                elif label == 'phoenix':
                    color = (0, 0, 255)
                elif label == 'way_point':
                    color = (0, 255, 0)
                else:
                    color = (200, 255, 200)
                text_item = pg.TextItem(text=label, color=color)
                ui_win.gps_graphicsView.addItem(text_item)
                gps_position_labels[label] = text_item
            gps_position_labels[label].setPos(gps_positions[label]['pos'][0], gps_positions[label]['pos'][1])
        # remove unused labels
        for label in gps_position_labels.keys():
            if label not in gps_positions.keys():
                text_item = gps_position_labels[label]
                ui_win.gps_graphicsView.removeItem(text_item)
                del gps_position_labels[label]


def callback_gps_home(cur_gps_input):
    gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
    if 'home' in gps_positions.keys():
        if gps_pos != gps_positions['home']['pos']:
            gps_positions['home']['pos'] = gps_pos
    else:
        gps_positions['home'] = {'pos': gps_pos, 'symbol': 'o', 'brush': pg.mkBrush(color=(255, 255, 0))}


def callback_gps_way_point(cur_gps_input):
    gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
    if 'way_point' in gps_positions.keys():
        if gps_pos != gps_positions['way_point']['pos']:
            gps_positions['way_point']['pos'] = gps_pos
    else:
        gps_positions['way_point'] = {'pos': gps_pos, 'symbol': 'o', 'brush': pg.mkBrush(color=(0, 255, 0))}


def callback_gps_position(cur_gps_input):
    if (len(gps_data[0]) == 0) or (cur_gps_input.longitude != gps_data[0][-1]) and (cur_gps_input.latitude != gps_data[1][-1]):
        gps_data[0].append(cur_gps_input.longitude)
        gps_data[1].append(cur_gps_input.latitude)

    gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
    if 'phoenix' in gps_positions.keys():
        if gps_pos != gps_positions['phoenix']['pos']:
            gps_positions['phoenix']['pos'] = gps_pos
    else:
        gps_positions['phoenix'] = {'pos': gps_pos, 'symbol': 'o', 'brush': pg.mkBrush(color=(0, 0, 255))}


def set_parameters_lcd(number, val=0):
    if number == 0:
        ui_win.lcdNumber_parameter_00.display(val)
    elif number == 1:
        ui_win.lcdNumber_parameter_01.display(val)
    elif number == 2:
        ui_win.lcdNumber_parameter_02.display(val)
    elif number == 3:
        ui_win.lcdNumber_parameter_03.display(val)
    elif number == 4:
        ui_win.lcdNumber_parameter_04.display(val)
    elif number == 5:
        ui_win.lcdNumber_parameter_05.display(val)
    elif number == 6:
        ui_win.lcdNumber_parameter_06.display(val)
    elif number == 7:
        ui_win.lcdNumber_parameter_07.display(val)
    elif number == 8:
        ui_win.lcdNumber_parameter_08.display(val)
    elif number == 9:
        ui_win.lcdNumber_parameter_09.display(val)
    elif number == 10:
        ui_win.lcdNumber_parameter_10.display(val)
    elif number == 11:
        ui_win.lcdNumber_parameter_11.display(val)
    elif number == 12:
        ui_win.lcdNumber_parameter_12.display(val)
    elif number == 13:
        ui_win.lcdNumber_parameter_13.display(val)
    elif number == 14:
        ui_win.lcdNumber_parameter_14.display(val)
    elif number == 15:
        ui_win.lcdNumber_parameter_15.display(val)
    elif number == 16:
        ui_win.lcdNumber_parameter_16.display(val)
    elif number == 17:
        ui_win.lcdNumber_parameter_17.display(val)
    else:
        print ' -> set_parameters_lcd requested number', number, 'not available'


def set_parameters_slider_limits(number, min=10, max=2000):
    if number == 0:
        ui_win.horizontalSlider_parameter_00.setRange(min, max)
    elif number == 1:
        ui_win.horizontalSlider_parameter_01.setRange(min, max)
    elif number == 2:
        ui_win.horizontalSlider_parameter_02.setRange(min, max)
    elif number == 3:
        ui_win.horizontalSlider_parameter_03.setRange(min, max)
    elif number == 4:
        ui_win.horizontalSlider_parameter_04.setRange(min, max)
    elif number == 5:
        ui_win.horizontalSlider_parameter_05.setRange(min, max)
    elif number == 6:
        ui_win.horizontalSlider_parameter_06.setRange(min, max)
    elif number == 7:
        ui_win.horizontalSlider_parameter_07.setRange(min, max)
    elif number == 8:
        ui_win.horizontalSlider_parameter_08.setRange(min, max)
    elif number == 9:
        ui_win.horizontalSlider_parameter_09.setRange(min, max)
    elif number == 10:
        ui_win.horizontalSlider_parameter_10.setRange(min, max)
    elif number == 11:
        ui_win.horizontalSlider_parameter_11.setRange(min, max)
    elif number == 12:
        ui_win.horizontalSlider_parameter_12.setRange(min, max)
    elif number == 13:
        ui_win.horizontalSlider_parameter_13.setRange(min, max)
    elif number == 14:
        ui_win.horizontalSlider_parameter_14.setRange(min, max)
    elif number == 15:
        ui_win.horizontalSlider_parameter_15.setRange(min, max)
    elif number == 16:
        ui_win.horizontalSlider_parameter_16.setRange(min, max)
    elif number == 17:
        ui_win.horizontalSlider_parameter_17.setRange(min, max)
    else:
        print ' -> set_parameters_slider_limits requested number', number, 'not available'


def set_parameters_slider(number, val, lcd_linked=True):
    if number == 0:
        ui_win.horizontalSlider_parameter_00.setValue(val)
    elif number == 1:
        ui_win.horizontalSlider_parameter_01.setValue(val)
    elif number == 2:
        ui_win.horizontalSlider_parameter_02.setValue(val)
    elif number == 3:
        ui_win.horizontalSlider_parameter_03.setValue(val)
    elif number == 4:
        ui_win.horizontalSlider_parameter_04.setValue(val)
    elif number == 5:
        ui_win.horizontalSlider_parameter_05.setValue(val)
    elif number == 6:
        ui_win.horizontalSlider_parameter_06.setValue(val)
    elif number == 7:
        ui_win.horizontalSlider_parameter_07.setValue(val)
    elif number == 8:
        ui_win.horizontalSlider_parameter_08.setValue(val)
    elif number == 9:
        ui_win.horizontalSlider_parameter_09.setValue(val)
    elif number == 10:
        ui_win.horizontalSlider_parameter_10.setValue(val)
    elif number == 11:
        ui_win.horizontalSlider_parameter_11.setValue(val)
    elif number == 12:
        ui_win.horizontalSlider_parameter_12.setValue(val)
    elif number == 13:
        ui_win.horizontalSlider_parameter_13.setValue(val)
    elif number == 14:
        ui_win.horizontalSlider_parameter_14.setValue(val)
    elif number == 15:
        ui_win.horizontalSlider_parameter_15.setValue(val)
    elif number == 16:
        ui_win.horizontalSlider_parameter_16.setValue(val)
    elif number == 17:
        ui_win.horizontalSlider_parameter_17.setValue(val)
    else:
        print ' -> set_parameters_slider requested number', number, 'not available'
    if lcd_linked:
        set_parameters_lcd(number, val)


def get_parameters_slider(number):
    if number == 0:
        return ui_win.horizontalSlider_parameter_00.value()
    elif number == 1:
        return ui_win.horizontalSlider_parameter_01.value()
    elif number == 2:
        return ui_win.horizontalSlider_parameter_02.value()
    elif number == 3:
        return ui_win.horizontalSlider_parameter_03.value()
    elif number == 4:
        return ui_win.horizontalSlider_parameter_04.value()
    elif number == 5:
        return ui_win.horizontalSlider_parameter_05.value()
    elif number == 6:
        return ui_win.horizontalSlider_parameter_06.value()
    elif number == 7:
        return ui_win.horizontalSlider_parameter_07.value()
    elif number == 8:
        return ui_win.horizontalSlider_parameter_08.value()
    elif number == 9:
        return ui_win.horizontalSlider_parameter_09.value()
    elif number == 10:
        return ui_win.horizontalSlider_parameter_10.value()
    elif number == 11:
        return ui_win.horizontalSlider_parameter_11.value()
    elif number == 12:
        return ui_win.horizontalSlider_parameter_12.value()
    elif number == 13:
        return ui_win.horizontalSlider_parameter_13.value()
    elif number == 14:
        return ui_win.horizontalSlider_parameter_14.value()
    elif number == 15:
        return ui_win.horizontalSlider_parameter_15.value()
    elif number == 16:
        return ui_win.horizontalSlider_parameter_16.value()
    elif number == 17:
        return ui_win.horizontalSlider_parameter_17.value()
    else:
        print ' -> get_parameters_slider requested number', number, 'not available'
        return False


def callback_cur_servo_cmd(cur_servo_cmd):
    set_parameters_slider(0, cur_servo_cmd.servo0)
    set_parameters_slider(1, cur_servo_cmd.servo1)
    set_parameters_slider(2, cur_servo_cmd.servo2)
    set_parameters_slider(3, cur_servo_cmd.servo3)
    set_parameters_slider(4, cur_servo_cmd.servo4)
    set_parameters_slider(5, cur_servo_cmd.servo5)
    set_parameters_slider(6, cur_servo_cmd.servo6)
    set_parameters_slider(7, cur_servo_cmd.servo7)
    set_parameters_slider(8, cur_servo_cmd.servo8)
    set_parameters_slider(9, cur_servo_cmd.servo9)
    set_parameters_slider(10, cur_servo_cmd.servo10)
    set_parameters_slider(11, cur_servo_cmd.servo11)
    set_parameters_slider(12, cur_servo_cmd.servo12)
    set_parameters_slider(13, cur_servo_cmd.servo13)
    set_parameters_slider(14, cur_servo_cmd.servo14)
    set_parameters_slider(15, cur_servo_cmd.servo15)
    set_parameters_slider(16, cur_servo_cmd.servo16)
    set_parameters_slider(17, cur_servo_cmd.servo17)
    print ' -> updated sliders from cur_servo_cmd'


rospy.init_node('gauge_gui')
ros_subscribe_cur_servo_cmd = rospy.Subscriber('/crab/uart_bridge/cur_servo_cmd', Servo, callback_cur_servo_cmd)
ros_subscribe_gps_position = rospy.Subscriber('/phx/gps', NavSatFix, callback_gps_position)
ros_subscribe_gps_way_point = rospy.Subscriber('/phx/fc/gps_way_point', NavSatFix, callback_gps_way_point)
update_interval = 100    # ms
publish_servo = True
ros_publisher_servo_cmd = rospy.Publisher('/crab/uart_bridge/servo_cmd', Servo, queue_size=1)

for i in range(0, 18):
    set_parameters_slider_limits(i, 300, 2450)


def mainloop():
    #parameters = [get_parameters_slider(i) for i in range(0, 18)]
    #for i in range(0, 18):
    #    set_parameters_lcd(i, parameters[i])
    global publish_servo
    if publish_servo:
        send_servos_msg = Servo()
        send_servos_msg.servo0 = get_parameters_slider(0)
        send_servos_msg.servo1 = get_parameters_slider(1)
        send_servos_msg.servo2 = get_parameters_slider(2)
        send_servos_msg.servo3 = get_parameters_slider(3)
        send_servos_msg.servo4 = get_parameters_slider(4)
        send_servos_msg.servo5 = get_parameters_slider(5)
        send_servos_msg.servo6 = get_parameters_slider(6)
        send_servos_msg.servo7 = get_parameters_slider(7)
        send_servos_msg.servo8 = get_parameters_slider(8)
        send_servos_msg.servo9 = get_parameters_slider(9)
        send_servos_msg.servo10 = get_parameters_slider(10)
        send_servos_msg.servo11 = get_parameters_slider(11)
        send_servos_msg.servo12 = get_parameters_slider(12)
        send_servos_msg.servo13 = get_parameters_slider(13)
        send_servos_msg.servo14 = get_parameters_slider(14)
        send_servos_msg.servo15 = get_parameters_slider(15)
        send_servos_msg.servo16 = get_parameters_slider(16)
        send_servos_msg.servo17 = get_parameters_slider(17)
        ros_publisher_servo_cmd.publish(send_servos_msg)
    print 'mainloop', win.keysPressed

    try:
        update_gps_plot(path=True, points=True)
    except:
        print '>>> error in main loop'
win.show()
# QTimer
timer = QtCore.QTimer()
timer.timeout.connect(mainloop)
interval_ms = update_interval
timer.start(interval_ms)

app.exec_()

print 'end'
timer.stop()

ros_subscribe_cur_servo_cmd.unregister()
ros_subscribe_gps_position.unregister()
ros_subscribe_gps_way_point.unregister()



