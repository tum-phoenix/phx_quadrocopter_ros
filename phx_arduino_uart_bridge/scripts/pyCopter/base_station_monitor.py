__author__ = 'manuelviermetz'

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg


class status_monitor():
    def __init__(self, osc_transmitter=None):
        self.osc_transmitter = osc_transmitter

        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        self.win = MainWindow()            # QtGui.QMainWindow()
        self.win.resize(1000, 800)
        self.win.setWindowTitle('LiveStatus')
        self.win.statusBar().showMessage("ready")
        self.cw = QtGui.QWidget()
        self.win.setCentralWidget(self.cw)
        self.l = QtGui.QGridLayout()
        self.cw.setLayout(self.l)

        # init data storage:
        self.status_data_keys = ['pitch', 'roll', 'yaw',
                                 'pitch_M', 'roll_M', 'yaw_M',
                                 'altitude',
                                 'cycletime_0', 'cycletime_1',
                                 '0_throttle', '0_pitch', '0_roll', '0_yaw',
                                 '0_aux1', '0_aux2', '0_aux3', '0_aux4',
                                 '1_throttle', '1_pitch', '1_roll', '1_yaw',
                                 '1_aux1', '1_aux2', '1_aux3', '1_aux4',
                                 '2_throttle', '2_pitch', '2_roll', '2_yaw',
                                 '2_aux1', '2_aux2', '2_aux3', '2_aux4',
                                 'accX', 'accY', 'accZ',
                                 'accX_M', 'accY_M', 'accZ_M',
                                 'gyrX', 'gyrY', 'gyrZ',
                                 'gyrX_M', 'gyrY_M', 'gyrZ_M',
                                 'magX', 'magY', 'magZ',
                                 'magX_M', 'magY_M', 'magZ_M',
                                 'motor_0', 'motor_1', 'motor_2', 'motor_3',
                                 'cell1', 'cell2', 'cell3', 'cell4']
        self.status_data = {}
        length_of_stored_data = 300
        for key in self.status_data_keys:
            self.status_data[key] = [0]*length_of_stored_data

        # generate layout segments
        pens = [pg.mkPen(width=1., color='r'), pg.mkPen(width=1., color='g'), pg.mkPen(width=1., color='b'), pg.mkPen(width=1., color='w')]
        colors = ['red', 'green', 'blue', 'grey']

        self.screens = [[['pitch', 'roll', 'yaw'                           ], 'widget', 'plots', 'data', 'labels'],
                        [['pitch_M', 'roll_M', 'yaw_M'                     ], 'widget', 'plots', 'data', 'labels'],
                        [['altitude'                                       ], 'widget', 'plots', 'data', 'labels'],
                        [['cycletime_0', 'cycletime_1'                     ], 'widget', 'plots', 'data', 'labels'],
                        [['cell1', 'cell2', 'cell3', 'cell4'               ], 'widget', 'plots', 'data', 'labels'],
                        [['0_throttle', '0_pitch', '0_roll', '0_yaw'       ], 'widget', 'plots', 'data', 'labels'],
                        [['1_throttle', '1_pitch', '1_roll', '1_yaw'       ], 'widget', 'plots', 'data', 'labels'],
                        [['2_throttle', '2_pitch', '2_roll', '2_yaw'       ], 'widget', 'plots', 'data', 'labels'],
                        [['motor_0', 'motor_1', 'motor_2', 'motor_3'       ], 'widget', 'plots', 'data', 'labels'],
                        [['accX', 'accY', 'accZ'], 'widget', 'plots', 'data', 'labels'],
                        [['gyrX', 'gyrY', 'gyrZ'], 'widget', 'plots', 'data', 'labels'],
                        [['magX', 'magY', 'magZ'], 'widget', 'plots', 'data', 'labels']]

        plot_height = 300
        box_width = 80
        collums = 4
        for screen_number in range(0, len(self.screens)):
            screen_config = self.screens[screen_number]
            number_of_plots = len(screen_config[0])
            widget = pg.PlotWidget()
            widget.setMaximumHeight(plot_height)
            widget.setMinimumHeight(plot_height)
            plots = []
            data_key = []
            for plot_number in range(0, number_of_plots):
                plots.append(widget.plot())
                plots[-1].setPen(pens[plot_number])
                data_key.append(screen_config[0][plot_number])

            box = QtGui.QGroupBox("monitor: "+str(screen_number))
            box.setMaximumWidth(box_width); box.setMinimumWidth(box_width)
            labels = {}
            for plot_number in range(0,number_of_plots):
                title = screen_config[0][plot_number]
                label1 = QtGui.QLabel(box)
                label1.move(5, 25+plot_number*30)
                label1.setText(title+":")
                label1.setStyleSheet("color: "+colors[plot_number])
                label2 = QtGui.QLabel(box)
                label2.move(5, 40+plot_number*30)
                label2.setText(" s       ")
                labels[title] = [label1, label2]
            self.screens[screen_number][1] = [widget, box]
            self.screens[screen_number][2] = plots
            self.screens[screen_number][3] = data_key
            self.screens[screen_number][4] = labels
            self.l.addWidget(box, int(screen_number/collums), (2*screen_number) % (collums*2))
            self.l.addWidget(widget, int((2*screen_number+1)/(collums*2)), (2*screen_number+1) % (collums*2))

        print self.screens[0][0]
        print self.screens[0][1]
        print self.screens[0][2]
        print self.screens[0][3]
        print self.screens[0][4]
        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.mainloop)
        self.interval = 1./30.
        self.timer.start(self.interval)

    def run(self):
        self.win.show()
        self.app.exec_()

    def handle_status_time(self, add, tag, stuff, source):
        print ' >>> TIME:', add, tag, stuff, source

    def handle_status_rc0(self, add, tag, stuff, source):
        """
            stuff = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        print ' >>> RC0:', add, tag, stuff, source
        stuff_labels = ['0_throttle', '0_pitch', '0_roll', '0_yaw', '0_aux1', '0_aux2', '0_aux3', '0_aux4']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_rc1(self, add, tag, stuff, source):
        """
            stuff = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        print ' >>> RC1:', add, tag, stuff, source
        stuff_labels = ['1_throttle', '1_pitch', '1_roll', '1_yaw', '1_aux1', '1_aux2', '1_aux3', '1_aux4']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_rc2(self, add, tag, stuff, source):
        """
            stuff = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        print ' >>> RC2:', add, tag, stuff, source
        stuff_labels = ['2_throttle', '2_pitch', '2_roll', '2_yaw', '2_aux1', '2_aux2', '2_aux3', '2_aux4']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_imu(self, add, tag, stuff, source):
        """
            imu = [ accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY. magZ, pitch, roll, yaw, altitude ]
        """
        print ' >>> IMU:', add, tag, stuff, source
        stuff_labels = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ', 'magX', 'magY', 'magZ', 'pitch', 'roll', 'yaw', 'altitude']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_imu_M(self, add, tag, stuff, source):
        """
            this is the imu of marvic!
            imu = [ accX_M, accY_M, accZ_M, gyrX_M, gyrY_M, gyrZ_M, magX_M, magY_M, magZ_M, pitch_M, roll_M, yaw_M, altitude_M ]
        """
        print ' >>> IMU:', add, tag, stuff, source
        stuff_labels = ['accX_M', 'accY_M', 'accZ_M', 'gyrX_M', 'gyrY_M', 'gyrZ_M', 'magX_M', 'magY_M', 'magZ_M', 'pitch_M', 'roll_M', 'yaw_M', 'altitude_M']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_cycletime0(self, add, tag, stuff, source):
        """
            cycletime0 = [cycletime]
        """
        print ' >>> cycletime0:', add, tag, stuff, source
        stuff_labels = ['cycletime_0']
        for i in range(0,len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_cycletime1(self, add, tag, stuff, source):
        """
            cycletime1 = [cycletime1]
        """
        print ' >>> cycletime1:', add, tag, stuff, source
        stuff_labels = ['cycletime_1']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_motors(self, add, tag, stuff, source):
        """
            motors = [ motor0, motor1, motor2, motor3 ]
        """
        print ' >>> motors:', add, tag, stuff, source
        stuff_labels = ['motor_0', 'motor_1', 'motor_2', 'motor_3']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    def handle_status_battery(self, add, tag, stuff, source):
        """
            battery = [ cell1, cell2, cell3, cell4 ]
        """
        print ' >>> battery:', add, tag, stuff, source
        stuff_labels = ['cell1', 'cell2', 'cell3', 'cell4']
        for i in range(0, len(stuff_labels)):
            self.status_data[stuff_labels[i]].pop(0)
            self.status_data[stuff_labels[i]].append(stuff[i])

    # ==============================================================================
    # MAINLOOP
    # ==============================================================================
    def mainloop(self):
        """ update GUI """
        for screen_number in range(0, len(self.screens)):
            screen_config = self.screens[screen_number]   # [ 'pitch','roll','yaw'] , [plot,box] , ['plots'] , [ 1 , 2 , 3] , {labels}
            labels = screen_config[4]
            for plot_number in range(0, len(screen_config[0])):
                data_key = screen_config[3][plot_number]
                data_name = screen_config[0][plot_number]
                data = self.status_data[data_key]
                screen_config[2][plot_number].setData(data)
                labels[data_name][1].setText(str(data[-1]).rjust(6))
        self.app.processEvents()

        """ handle keyboard input """
        # space 32
        # left 16777234
        # right 16777236
        # up 16777235
        # down 16777237
        if QtCore.Qt.Key_Escape in self.win.keysPressed:
            self.app.exit()
        if self.osc_transmitter:
            directions = [0, 0, 0, 0]                                           # backward-forward, left-right, up-down, left-right-turn
            if QtCore.Qt.Key_Up in self.win.keysPressed:
                directions[0] += 1
            if QtCore.Qt.Key_Down in self.win.keysPressed:
                directions[0] -= 1
            if QtCore.Qt.Key_Right in self.win.keysPressed:
                directions[1] += 1
            if QtCore.Qt.Key_Left in self.win.keysPressed:
                directions[1] -= 1
            if QtCore.Qt.Key_W in self.win.keysPressed:
                directions[2] += 1
            if QtCore.Qt.Key_S in self.win.keysPressed:
                directions[2] -= 1
            if QtCore.Qt.Key_A in self.win.keysPressed:
                directions[1] -= 1
            if QtCore.Qt.Key_D in self.win.keysPressed:
                directions[1] += 1
            if QtCore.Qt.Key_Q in self.win.keysPressed:
                directions[3] -= 1
            if QtCore.Qt.Key_E in self.win.keysPressed:
                directions[3] += 1
            self.osc_transmitter.send_keyboard_directions(directions)
            if QtCore.Qt.Key_C in self.win.keysPressed:                     # on KeyPress [c] sending calibrate_accelerometer
                self.osc_transmitter.send_keyboard_commands(['cal_acc'])
            if QtCore.Qt.Key_1 in self.win.keysPressed:                     # on KeyPress [1] sending pLED = 1
                self.osc_transmitter.send_keyboard_command(['pLED=1'])
            if QtCore.Qt.Key_2 in self.win.keysPressed:                     # on KeyPress [2] sending pLED = 2
                self.osc_transmitter.send_keyboard_command(['pLED=2'])
            if QtCore.Qt.Key_3 in self.win.keysPressed:                     # on KeyPress [3] sending pLED = 3
                self.osc_transmitter.send_keyboard_command(['pLED=3'])
            if QtCore.Qt.Key_4 in self.win.keysPressed:                     # on KeyPress [4] sending pLED = 4
                self.osc_transmitter.send_keyboard_command(['pLED=4'])


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
#        if ev.isAutoRepeat():
#            return
        self.mouseClicks.append([ev.x(), ev.y(), ev.pos(), ev.button()])

    def keyPressEvent(self, ev):
        # print ev.key(), QtCore.Qt.Key_Up
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