import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

import rospy
from phx_uart_msp_bridge.msg import Diagnostics



win = pg.GraphicsWindow()
win.setWindowTitle('diagnostics gui')

# Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100
# Remove chunks after we have 10
maxChunks = 10
startTime = pg.ptime.time()

plot_0 = win.addPlot(colspan=2)
plot_0.setLabel('bottom', 'Time', 's')
plot_0.setXRange(-10, 0)
curves_0 = [[], [], []]
data_0 = np.empty((chunkSize+1, 4))
ptr_0 = 0

win.nextRow()
plot_1 = win.addPlot(colspan=2)
plot_1.setLabel('bottom', 'Time', 's')
plot_1.setXRange(-10, 0)
curves_1 = [[], [], []]
data_1 = np.empty((chunkSize+1, 4))
ptr_1 = 0

win.nextRow()
plot_2 = win.addPlot(colspan=2)
plot_2.setLabel('bottom', 'Time', 's')
plot_2.setXRange(-10, 0)
curves_2 = [[], [], []]
data_2 = np.empty((chunkSize+1, 4))
ptr_2 = 0


def update_plot(time=None,
                new_val_0=None,
                new_val_1=None,
                new_val_2=None):
    global plot_0, plot_1, plot_2
    global data_0, data_1, data_2
    global ptr_0, ptr_1, ptr_2
    global curves_0, curves_1, curves_2

    if time is None:
        now = pg.ptime.time()
    else:
        now = time
    # update plot 0
    for c in curves_0:
        for cc in c:
            cc.setPos(-(now-startTime), 0)

    i = ptr_0 % chunkSize
    if i == 0:
        curve_i = plot_0.plot(pen='r')
        curve_ii = plot_0.plot(pen='g')
        curve_iii = plot_0.plot(pen='b')
        curves_0.append([curve_i, curve_ii, curve_iii])
        curve = curves_0[-1]
        last = data_0[-1]
        data_0 = np.empty((chunkSize+1, 4))
        data_0[0] = last
        while len(curves_0) > maxChunks:
            c = curves_0.pop(0)
            plot_0.removeItem(c[0])
            plot_0.removeItem(c[1])
            plot_0.removeItem(c[2])
    else:
        curve = curves_0[-1]
    data_0[i+1, 0] = now - startTime
    data_0[i+1, 1] = new_val_0[0]
    data_0[i+1, 2] = new_val_0[1]
    data_0[i+1, 3] = new_val_0[2]
    curve[0].setData(x=data_0[:i+2, 0], y=data_0[:i+2, 1])
    curve[1].setData(x=data_0[:i+2, 0], y=data_0[:i+2, 2])
    curve[2].setData(x=data_0[:i+2, 0], y=data_0[:i+2, 3])
    ptr_0 += 1

    # update plot 1
    for c in curves_1:
        for cc in c:
            cc.setPos(-(now - startTime), 0)

    i = ptr_1 % chunkSize
    if i == 0:
        curve_i = plot_1.plot(pen='r')
        curve_ii = plot_1.plot(pen='g')
        curve_iii = plot_1.plot(pen='b')
        curves_1.append([curve_i, curve_ii, curve_iii])
        curve = curves_1[-1]
        last = data_1[-1]
        data_1 = np.empty((chunkSize + 1, 4))
        data_1[0] = last
        while len(curves_1) > maxChunks:
            c = curves_1.pop(0)
            plot_1.removeItem(c[0])
            plot_1.removeItem(c[1])
            plot_1.removeItem(c[2])
    else:
        curve = curves_1[-1]
    data_1[i + 1, 0] = now - startTime
    data_1[i + 1, 1] = new_val_1[0]
    data_1[i + 1, 2] = new_val_1[1]
    data_1[i + 1, 3] = new_val_1[2]
    curve[0].setData(x=data_1[:i + 2, 0], y=data_1[:i + 2, 1])
    curve[1].setData(x=data_1[:i + 2, 0], y=data_1[:i + 2, 2])
    curve[2].setData(x=data_1[:i + 2, 0], y=data_1[:i + 2, 3])
    ptr_1 += 1

    # update plot 2
    for c in curves_2:
        for cc in c:
            cc.setPos(-(now - startTime), 0)

    i = ptr_2 % chunkSize
    if i == 0:
        curve_i = plot_2.plot(pen='r')
        curve_ii = plot_2.plot(pen='g')
        curve_iii = plot_2.plot(pen='b')
        curves_2.append([curve_i, curve_ii, curve_iii])
        curve = curves_2[-1]
        last = data_2[-1]
        data_2 = np.empty((chunkSize + 1, 4))
        data_2[0] = last
        while len(curves_2) > maxChunks:
            c = curves_2.pop(0)
            plot_2.removeItem(c[0])
            plot_2.removeItem(c[1])
            plot_2.removeItem(c[2])
    else:
        curve = curves_2[-1]
    data_2[i + 1, 0] = now - startTime
    data_2[i + 1, 1] = new_val_2[0]
    data_2[i + 1, 2] = new_val_2[1]
    data_2[i + 1, 3] = new_val_2[2]
    curve[0].setData(x=data_2[:i + 2, 0], y=data_2[:i + 2, 1])
    curve[1].setData(x=data_2[:i + 2, 0], y=data_2[:i + 2, 2])
    curve[2].setData(x=data_2[:i + 2, 0], y=data_2[:i + 2, 3])
    ptr_2 += 1


# update all plots for test
# def random_noise_test():
#     update_plot(new_val_0=[np.random.normal(), np.random.normal(), np.random.normal()],
#                 new_val_1=[np.random.normal(), np.random.normal(), np.random.normal()],
#                 new_val_2=[np.random.normal(), np.random.normal(), np.random.normal()])
#
# timer = pg.QtCore.QTimer()
# timer.timeout.connect(random_noise_test)
# timer.start(50)

def update_from_diagnostics_channel(input_diagnostics):
    print 'received diagnostics update'
    time_from_header = input_diagnostics.header.stamp.secs
    update_plot(time=time_from_header,
                new_val_0=[input_diagnostics.val_a0, input_diagnostics.val_a1, input_diagnostics.val_a2],
                new_val_1=[input_diagnostics.val_b0, input_diagnostics.val_b1, input_diagnostics.val_b2],
                new_val_2=[input_diagnostics.val_c0, input_diagnostics.val_c1, input_diagnostics.val_c2])
    print time_from_header

rospy.init_node('diagnostics_gui')

ros_subscribe_cur_servo_cmd = rospy.Subscriber('/diagnostics', Diagnostics, update_from_diagnostics_channel)


QtGui.QApplication.instance().exec_()


