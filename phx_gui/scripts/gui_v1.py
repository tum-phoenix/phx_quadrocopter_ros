# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui_v1.ui'
#
# Created: Fri Sep 25 00:36:42 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1302, 730)
        MainWindow.setTabShape(QtGui.QTabWidget.Rounded)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 1276, 661))
        self.horizontalLayoutWidget.setObjectName(_fromUtf8("horizontalLayoutWidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setMargin(0)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.left_tabs = QtGui.QTabWidget(self.horizontalLayoutWidget)
        self.left_tabs.setObjectName(_fromUtf8("left_tabs"))
        self.InfoPage = QtGui.QWidget()
        self.InfoPage.setObjectName(_fromUtf8("InfoPage"))
        self.verticalLayout_plots = QtGui.QVBoxLayout(self.InfoPage)
        self.verticalLayout_plots.setObjectName(_fromUtf8("verticalLayout_plots"))
        self.textBrowser = QtGui.QTextBrowser(self.InfoPage)
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.verticalLayout_plots.addWidget(self.textBrowser)
        self.left_tabs.addTab(self.InfoPage, _fromUtf8(""))
        self.gpsPage = QtGui.QWidget()
        self.gpsPage.setObjectName(_fromUtf8("gpsPage"))
        self.gps_graphicsView = PlotWidget(self.gpsPage)
        self.gps_graphicsView.setGeometry(QtCore.QRect(20, 20, 591, 491))
        self.gps_graphicsView.setObjectName(_fromUtf8("gps_graphicsView"))
        self.left_tabs.addTab(self.gpsPage, _fromUtf8(""))
        self.ledPage = QtGui.QWidget()
        self.ledPage.setObjectName(_fromUtf8("ledPage"))
        self.gridLayoutWidget = QtGui.QWidget(self.ledPage)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(70, 60, 160, 101))
        self.gridLayoutWidget.setObjectName(_fromUtf8("gridLayoutWidget"))
        self.gridLayout_led_strip0 = QtGui.QGridLayout(self.gridLayoutWidget)
        self.gridLayout_led_strip0.setMargin(0)
        self.gridLayout_led_strip0.setObjectName(_fromUtf8("gridLayout_led_strip0"))
        self.label_3 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout_led_strip0.addWidget(self.label_3, 2, 0, 1, 1)
        self.label = QtGui.QLabel(self.gridLayoutWidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout_led_strip0.addWidget(self.label, 0, 0, 1, 1)
        self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout_led_strip0.addWidget(self.label_2, 1, 0, 1, 1)
        self.horizontalSlider_led_0_r = QtGui.QSlider(self.gridLayoutWidget)
        self.horizontalSlider_led_0_r.setMaximum(255)
        self.horizontalSlider_led_0_r.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_0_r.setObjectName(_fromUtf8("horizontalSlider_led_0_r"))
        self.gridLayout_led_strip0.addWidget(self.horizontalSlider_led_0_r, 0, 1, 1, 1)
        self.horizontalSlider_led_0_g = QtGui.QSlider(self.gridLayoutWidget)
        self.horizontalSlider_led_0_g.setMaximum(255)
        self.horizontalSlider_led_0_g.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_0_g.setObjectName(_fromUtf8("horizontalSlider_led_0_g"))
        self.gridLayout_led_strip0.addWidget(self.horizontalSlider_led_0_g, 1, 1, 1, 1)
        self.horizontalSlider_led_0_b = QtGui.QSlider(self.gridLayoutWidget)
        self.horizontalSlider_led_0_b.setMaximum(255)
        self.horizontalSlider_led_0_b.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_0_b.setObjectName(_fromUtf8("horizontalSlider_led_0_b"))
        self.gridLayout_led_strip0.addWidget(self.horizontalSlider_led_0_b, 2, 1, 1, 1)
        self.led_label_0 = QtGui.QLabel(self.ledPage)
        self.led_label_0.setGeometry(QtCore.QRect(70, 40, 91, 17))
        self.led_label_0.setObjectName(_fromUtf8("led_label_0"))
        self.gridLayoutWidget_2 = QtGui.QWidget(self.ledPage)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(270, 60, 160, 101))
        self.gridLayoutWidget_2.setObjectName(_fromUtf8("gridLayoutWidget_2"))
        self.gridLayout_led_strip1 = QtGui.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_led_strip1.setMargin(0)
        self.gridLayout_led_strip1.setObjectName(_fromUtf8("gridLayout_led_strip1"))
        self.label_8 = QtGui.QLabel(self.gridLayoutWidget_2)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout_led_strip1.addWidget(self.label_8, 2, 0, 1, 1)
        self.label_9 = QtGui.QLabel(self.gridLayoutWidget_2)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_led_strip1.addWidget(self.label_9, 0, 0, 1, 1)
        self.label_10 = QtGui.QLabel(self.gridLayoutWidget_2)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_led_strip1.addWidget(self.label_10, 1, 0, 1, 1)
        self.horizontalSlider_led_1_r = QtGui.QSlider(self.gridLayoutWidget_2)
        self.horizontalSlider_led_1_r.setMaximum(255)
        self.horizontalSlider_led_1_r.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_1_r.setObjectName(_fromUtf8("horizontalSlider_led_1_r"))
        self.gridLayout_led_strip1.addWidget(self.horizontalSlider_led_1_r, 0, 1, 1, 1)
        self.horizontalSlider_led_1_g = QtGui.QSlider(self.gridLayoutWidget_2)
        self.horizontalSlider_led_1_g.setMaximum(255)
        self.horizontalSlider_led_1_g.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_1_g.setObjectName(_fromUtf8("horizontalSlider_led_1_g"))
        self.gridLayout_led_strip1.addWidget(self.horizontalSlider_led_1_g, 1, 1, 1, 1)
        self.horizontalSlider_led_1_b = QtGui.QSlider(self.gridLayoutWidget_2)
        self.horizontalSlider_led_1_b.setMaximum(255)
        self.horizontalSlider_led_1_b.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_1_b.setObjectName(_fromUtf8("horizontalSlider_led_1_b"))
        self.gridLayout_led_strip1.addWidget(self.horizontalSlider_led_1_b, 2, 1, 1, 1)
        self.led_label_1 = QtGui.QLabel(self.ledPage)
        self.led_label_1.setGeometry(QtCore.QRect(270, 40, 91, 17))
        self.led_label_1.setObjectName(_fromUtf8("led_label_1"))
        self.gridLayoutWidget_3 = QtGui.QWidget(self.ledPage)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(70, 200, 160, 101))
        self.gridLayoutWidget_3.setObjectName(_fromUtf8("gridLayoutWidget_3"))
        self.gridLayout_led_strip2 = QtGui.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_led_strip2.setMargin(0)
        self.gridLayout_led_strip2.setObjectName(_fromUtf8("gridLayout_led_strip2"))
        self.label_11 = QtGui.QLabel(self.gridLayoutWidget_3)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_led_strip2.addWidget(self.label_11, 2, 0, 1, 1)
        self.label_12 = QtGui.QLabel(self.gridLayoutWidget_3)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_led_strip2.addWidget(self.label_12, 0, 0, 1, 1)
        self.label_13 = QtGui.QLabel(self.gridLayoutWidget_3)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_led_strip2.addWidget(self.label_13, 1, 0, 1, 1)
        self.horizontalSlider_led_2_r = QtGui.QSlider(self.gridLayoutWidget_3)
        self.horizontalSlider_led_2_r.setMaximum(255)
        self.horizontalSlider_led_2_r.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_2_r.setObjectName(_fromUtf8("horizontalSlider_led_2_r"))
        self.gridLayout_led_strip2.addWidget(self.horizontalSlider_led_2_r, 0, 1, 1, 1)
        self.horizontalSlider_led_2_g = QtGui.QSlider(self.gridLayoutWidget_3)
        self.horizontalSlider_led_2_g.setMaximum(255)
        self.horizontalSlider_led_2_g.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_2_g.setObjectName(_fromUtf8("horizontalSlider_led_2_g"))
        self.gridLayout_led_strip2.addWidget(self.horizontalSlider_led_2_g, 1, 1, 1, 1)
        self.horizontalSlider_led_2_b = QtGui.QSlider(self.gridLayoutWidget_3)
        self.horizontalSlider_led_2_b.setMaximum(255)
        self.horizontalSlider_led_2_b.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_2_b.setObjectName(_fromUtf8("horizontalSlider_led_2_b"))
        self.gridLayout_led_strip2.addWidget(self.horizontalSlider_led_2_b, 2, 1, 1, 1)
        self.led_label_2 = QtGui.QLabel(self.ledPage)
        self.led_label_2.setGeometry(QtCore.QRect(70, 180, 91, 17))
        self.led_label_2.setObjectName(_fromUtf8("led_label_2"))
        self.led_label_3 = QtGui.QLabel(self.ledPage)
        self.led_label_3.setGeometry(QtCore.QRect(270, 180, 91, 17))
        self.led_label_3.setObjectName(_fromUtf8("led_label_3"))
        self.gridLayoutWidget_4 = QtGui.QWidget(self.ledPage)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(270, 200, 160, 101))
        self.gridLayoutWidget_4.setObjectName(_fromUtf8("gridLayoutWidget_4"))
        self.gridLayout_led_strip3 = QtGui.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_led_strip3.setMargin(0)
        self.gridLayout_led_strip3.setObjectName(_fromUtf8("gridLayout_led_strip3"))
        self.label_14 = QtGui.QLabel(self.gridLayoutWidget_4)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.gridLayout_led_strip3.addWidget(self.label_14, 2, 0, 1, 1)
        self.label_15 = QtGui.QLabel(self.gridLayoutWidget_4)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_led_strip3.addWidget(self.label_15, 0, 0, 1, 1)
        self.label_16 = QtGui.QLabel(self.gridLayoutWidget_4)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_led_strip3.addWidget(self.label_16, 1, 0, 1, 1)
        self.horizontalSlider_led_3_r = QtGui.QSlider(self.gridLayoutWidget_4)
        self.horizontalSlider_led_3_r.setMaximum(255)
        self.horizontalSlider_led_3_r.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_3_r.setObjectName(_fromUtf8("horizontalSlider_led_3_r"))
        self.gridLayout_led_strip3.addWidget(self.horizontalSlider_led_3_r, 0, 1, 1, 1)
        self.horizontalSlider_led_3_g = QtGui.QSlider(self.gridLayoutWidget_4)
        self.horizontalSlider_led_3_g.setMaximum(255)
        self.horizontalSlider_led_3_g.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_3_g.setObjectName(_fromUtf8("horizontalSlider_led_3_g"))
        self.gridLayout_led_strip3.addWidget(self.horizontalSlider_led_3_g, 1, 1, 1, 1)
        self.horizontalSlider_led_3_b = QtGui.QSlider(self.gridLayoutWidget_4)
        self.horizontalSlider_led_3_b.setMaximum(255)
        self.horizontalSlider_led_3_b.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_led_3_b.setObjectName(_fromUtf8("horizontalSlider_led_3_b"))
        self.gridLayout_led_strip3.addWidget(self.horizontalSlider_led_3_b, 2, 1, 1, 1)
        self.pushButton_led_strip_update = QtGui.QPushButton(self.ledPage)
        self.pushButton_led_strip_update.setGeometry(QtCore.QRect(470, 270, 131, 27))
        self.pushButton_led_strip_update.setObjectName(_fromUtf8("pushButton_led_strip_update"))
        self.checkBox_led_strip_update_continuous = QtGui.QCheckBox(self.ledPage)
        self.checkBox_led_strip_update_continuous.setGeometry(QtCore.QRect(470, 240, 131, 22))
        self.checkBox_led_strip_update_continuous.setObjectName(_fromUtf8("checkBox_led_strip_update_continuous"))
        self.left_tabs.addTab(self.ledPage, _fromUtf8(""))
        self.horizontalLayout.addWidget(self.left_tabs)
        self.right_tabs = QtGui.QTabWidget(self.horizontalLayoutWidget)
        self.right_tabs.setObjectName(_fromUtf8("right_tabs"))
        self.parameter_tab = QtGui.QWidget()
        self.parameter_tab.setObjectName(_fromUtf8("parameter_tab"))
        self.widget = QtGui.QWidget(self.parameter_tab)
        self.widget.setGeometry(QtCore.QRect(10, 10, 611, 561))
        self.widget.setObjectName(_fromUtf8("widget"))
        self.gridLayout_parameters = QtGui.QGridLayout(self.widget)
        self.gridLayout_parameters.setSizeConstraint(QtGui.QLayout.SetMaximumSize)
        self.gridLayout_parameters.setMargin(0)
        self.gridLayout_parameters.setObjectName(_fromUtf8("gridLayout_parameters"))
        self.lcdNumber_parameter_00 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_00.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_00.setObjectName(_fromUtf8("lcdNumber_parameter_00"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_00, 0, 1, 1, 1)
        self.label_parameter_10 = QtGui.QLabel(self.widget)
        self.label_parameter_10.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_10.setObjectName(_fromUtf8("label_parameter_10"))
        self.gridLayout_parameters.addWidget(self.label_parameter_10, 10, 2, 1, 1)
        self.lcdNumber_parameter_16 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_16.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_16.setObjectName(_fromUtf8("lcdNumber_parameter_16"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_16, 16, 1, 1, 1)
        self.lcdNumber_parameter_06 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_06.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_06.setObjectName(_fromUtf8("lcdNumber_parameter_06"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_06, 6, 1, 1, 1)
        self.horizontalSlider_parameter_10 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_10.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_10.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_10.setObjectName(_fromUtf8("horizontalSlider_parameter_10"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_10, 10, 0, 1, 1)
        self.horizontalSlider_parameter_15 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_15.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_15.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_15.setObjectName(_fromUtf8("horizontalSlider_parameter_15"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_15, 15, 0, 1, 1)
        self.horizontalSlider_parameter_03 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_03.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_03.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_03.setObjectName(_fromUtf8("horizontalSlider_parameter_03"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_03, 3, 0, 1, 1)
        self.lcdNumber_parameter_01 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_01.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_01.setObjectName(_fromUtf8("lcdNumber_parameter_01"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_01, 1, 1, 1, 1)
        self.lcdNumber_parameter_03 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_03.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_03.setObjectName(_fromUtf8("lcdNumber_parameter_03"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_03, 3, 1, 1, 1)
        self.horizontalSlider_parameter_05 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_05.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_05.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_05.setObjectName(_fromUtf8("horizontalSlider_parameter_05"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_05, 5, 0, 1, 1)
        self.label_parameter_00 = QtGui.QLabel(self.widget)
        self.label_parameter_00.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_00.setObjectName(_fromUtf8("label_parameter_00"))
        self.gridLayout_parameters.addWidget(self.label_parameter_00, 0, 2, 1, 1)
        self.label_parameter_15 = QtGui.QLabel(self.widget)
        self.label_parameter_15.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_15.setObjectName(_fromUtf8("label_parameter_15"))
        self.gridLayout_parameters.addWidget(self.label_parameter_15, 15, 2, 1, 1)
        self.horizontalSlider_parameter_01 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_01.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_01.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_01.setObjectName(_fromUtf8("horizontalSlider_parameter_01"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_01, 1, 0, 1, 1)
        self.label_parameter_03 = QtGui.QLabel(self.widget)
        self.label_parameter_03.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_03.setObjectName(_fromUtf8("label_parameter_03"))
        self.gridLayout_parameters.addWidget(self.label_parameter_03, 3, 2, 1, 1)
        self.label_parameter_01 = QtGui.QLabel(self.widget)
        self.label_parameter_01.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_01.setObjectName(_fromUtf8("label_parameter_01"))
        self.gridLayout_parameters.addWidget(self.label_parameter_01, 1, 2, 1, 1)
        self.horizontalSlider_parameter_00 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_00.setMaximumSize(QtCore.QSize(1000, 20))
        self.horizontalSlider_parameter_00.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_00.setObjectName(_fromUtf8("horizontalSlider_parameter_00"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_00, 0, 0, 1, 1)
        self.horizontalSlider_parameter_07 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_07.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_07.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_07.setObjectName(_fromUtf8("horizontalSlider_parameter_07"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_07, 7, 0, 1, 1)
        self.horizontalSlider_parameter_02 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_02.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_02.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_02.setObjectName(_fromUtf8("horizontalSlider_parameter_02"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_02, 2, 0, 1, 1)
        self.label_parameter_04 = QtGui.QLabel(self.widget)
        self.label_parameter_04.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_04.setObjectName(_fromUtf8("label_parameter_04"))
        self.gridLayout_parameters.addWidget(self.label_parameter_04, 4, 2, 1, 1)
        self.horizontalSlider_parameter_04 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_04.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_04.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_04.setObjectName(_fromUtf8("horizontalSlider_parameter_04"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_04, 4, 0, 1, 1)
        self.label_parameter_11 = QtGui.QLabel(self.widget)
        self.label_parameter_11.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_11.setObjectName(_fromUtf8("label_parameter_11"))
        self.gridLayout_parameters.addWidget(self.label_parameter_11, 11, 2, 1, 1)
        self.horizontalSlider_parameter_12 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_12.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_12.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_12.setObjectName(_fromUtf8("horizontalSlider_parameter_12"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_12, 12, 0, 1, 1)
        self.horizontalSlider_parameter_16 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_16.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_16.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_16.setObjectName(_fromUtf8("horizontalSlider_parameter_16"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_16, 16, 0, 1, 1)
        self.label_parameter_06 = QtGui.QLabel(self.widget)
        self.label_parameter_06.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_06.setObjectName(_fromUtf8("label_parameter_06"))
        self.gridLayout_parameters.addWidget(self.label_parameter_06, 6, 2, 1, 1)
        self.horizontalSlider_parameter_06 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_06.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_06.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_06.setObjectName(_fromUtf8("horizontalSlider_parameter_06"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_06, 6, 0, 1, 1)
        self.label_parameter_07 = QtGui.QLabel(self.widget)
        self.label_parameter_07.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_07.setObjectName(_fromUtf8("label_parameter_07"))
        self.gridLayout_parameters.addWidget(self.label_parameter_07, 7, 2, 1, 1)
        self.lcdNumber_parameter_02 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_02.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_02.setObjectName(_fromUtf8("lcdNumber_parameter_02"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_02, 2, 1, 1, 1)
        self.label_parameter_02 = QtGui.QLabel(self.widget)
        self.label_parameter_02.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_02.setObjectName(_fromUtf8("label_parameter_02"))
        self.gridLayout_parameters.addWidget(self.label_parameter_02, 2, 2, 1, 1)
        self.lcdNumber_parameter_04 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_04.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_04.setObjectName(_fromUtf8("lcdNumber_parameter_04"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_04, 4, 1, 1, 1)
        self.lcdNumber_parameter_07 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_07.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_07.setObjectName(_fromUtf8("lcdNumber_parameter_07"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_07, 7, 1, 1, 1)
        self.lcdNumber_parameter_05 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_05.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_05.setObjectName(_fromUtf8("lcdNumber_parameter_05"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_05, 5, 1, 1, 1)
        self.label_parameter_05 = QtGui.QLabel(self.widget)
        self.label_parameter_05.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_05.setObjectName(_fromUtf8("label_parameter_05"))
        self.gridLayout_parameters.addWidget(self.label_parameter_05, 5, 2, 1, 1)
        self.horizontalSlider_parameter_08 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_08.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_08.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_08.setObjectName(_fromUtf8("horizontalSlider_parameter_08"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_08, 8, 0, 1, 1)
        self.label_parameter_09 = QtGui.QLabel(self.widget)
        self.label_parameter_09.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_09.setObjectName(_fromUtf8("label_parameter_09"))
        self.gridLayout_parameters.addWidget(self.label_parameter_09, 9, 2, 1, 1)
        self.label_parameter_08 = QtGui.QLabel(self.widget)
        self.label_parameter_08.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_08.setObjectName(_fromUtf8("label_parameter_08"))
        self.gridLayout_parameters.addWidget(self.label_parameter_08, 8, 2, 1, 1)
        self.horizontalSlider_parameter_13 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_13.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_13.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_13.setObjectName(_fromUtf8("horizontalSlider_parameter_13"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_13, 13, 0, 1, 1)
        self.label_parameter_13 = QtGui.QLabel(self.widget)
        self.label_parameter_13.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_13.setObjectName(_fromUtf8("label_parameter_13"))
        self.gridLayout_parameters.addWidget(self.label_parameter_13, 13, 2, 1, 1)
        self.horizontalSlider_parameter_11 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_11.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_11.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_11.setObjectName(_fromUtf8("horizontalSlider_parameter_11"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_11, 11, 0, 1, 1)
        self.label_parameter_14 = QtGui.QLabel(self.widget)
        self.label_parameter_14.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_14.setObjectName(_fromUtf8("label_parameter_14"))
        self.gridLayout_parameters.addWidget(self.label_parameter_14, 14, 2, 1, 1)
        self.label_parameter_12 = QtGui.QLabel(self.widget)
        self.label_parameter_12.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_12.setObjectName(_fromUtf8("label_parameter_12"))
        self.gridLayout_parameters.addWidget(self.label_parameter_12, 12, 2, 1, 1)
        self.horizontalSlider_parameter_14 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_14.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_14.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_14.setObjectName(_fromUtf8("horizontalSlider_parameter_14"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_14, 14, 0, 1, 1)
        self.horizontalSlider_parameter_09 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_09.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_09.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_09.setObjectName(_fromUtf8("horizontalSlider_parameter_09"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_09, 9, 0, 1, 1)
        self.horizontalSlider_parameter_17 = QtGui.QSlider(self.widget)
        self.horizontalSlider_parameter_17.setMaximumSize(QtCore.QSize(16777215, 20))
        self.horizontalSlider_parameter_17.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_parameter_17.setObjectName(_fromUtf8("horizontalSlider_parameter_17"))
        self.gridLayout_parameters.addWidget(self.horizontalSlider_parameter_17, 17, 0, 1, 1)
        self.label_parameter_16 = QtGui.QLabel(self.widget)
        self.label_parameter_16.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_16.setObjectName(_fromUtf8("label_parameter_16"))
        self.gridLayout_parameters.addWidget(self.label_parameter_16, 16, 2, 1, 1)
        self.label_parameter_17 = QtGui.QLabel(self.widget)
        self.label_parameter_17.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_parameter_17.setObjectName(_fromUtf8("label_parameter_17"))
        self.gridLayout_parameters.addWidget(self.label_parameter_17, 17, 2, 1, 1)
        self.lcdNumber_parameter_08 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_08.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_08.setObjectName(_fromUtf8("lcdNumber_parameter_08"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_08, 8, 1, 1, 1)
        self.lcdNumber_parameter_09 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_09.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_09.setObjectName(_fromUtf8("lcdNumber_parameter_09"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_09, 9, 1, 1, 1)
        self.lcdNumber_parameter_10 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_10.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_10.setObjectName(_fromUtf8("lcdNumber_parameter_10"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_10, 10, 1, 1, 1)
        self.lcdNumber_parameter_11 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_11.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_11.setObjectName(_fromUtf8("lcdNumber_parameter_11"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_11, 11, 1, 1, 1)
        self.lcdNumber_parameter_12 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_12.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_12.setObjectName(_fromUtf8("lcdNumber_parameter_12"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_12, 12, 1, 1, 1)
        self.lcdNumber_parameter_13 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_13.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_13.setObjectName(_fromUtf8("lcdNumber_parameter_13"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_13, 13, 1, 1, 1)
        self.lcdNumber_parameter_14 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_14.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_14.setObjectName(_fromUtf8("lcdNumber_parameter_14"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_14, 14, 1, 1, 1)
        self.lcdNumber_parameter_15 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_15.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_15.setObjectName(_fromUtf8("lcdNumber_parameter_15"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_15, 15, 1, 1, 1)
        self.lcdNumber_parameter_17 = QtGui.QLCDNumber(self.widget)
        self.lcdNumber_parameter_17.setMaximumSize(QtCore.QSize(16777215, 20))
        self.lcdNumber_parameter_17.setObjectName(_fromUtf8("lcdNumber_parameter_17"))
        self.gridLayout_parameters.addWidget(self.lcdNumber_parameter_17, 17, 1, 1, 1)
        self.checkBox_parameters_update_continuous = QtGui.QCheckBox(self.parameter_tab)
        self.checkBox_parameters_update_continuous.setGeometry(QtCore.QRect(300, 590, 131, 22))
        self.checkBox_parameters_update_continuous.setObjectName(_fromUtf8("checkBox_parameters_update_continuous"))
        self.pushButton_parameters_update = QtGui.QPushButton(self.parameter_tab)
        self.pushButton_parameters_update.setGeometry(QtCore.QRect(480, 590, 131, 27))
        self.pushButton_parameters_update.setObjectName(_fromUtf8("pushButton_parameters_update"))
        self.right_tabs.addTab(self.parameter_tab, _fromUtf8(""))
        self.altitude_tab = QtGui.QWidget()
        self.altitude_tab.setObjectName(_fromUtf8("altitude_tab"))
        self.graphicsView = PlotWidget(self.altitude_tab)
        self.graphicsView.setGeometry(QtCore.QRect(30, 10, 451, 371))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.right_tabs.addTab(self.altitude_tab, _fromUtf8(""))
        self.horizontalLayout.addWidget(self.right_tabs)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1302, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.left_tabs.setCurrentIndex(1)
        self.right_tabs.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "phoenixGUI", None))
        self.left_tabs.setTabText(self.left_tabs.indexOf(self.InfoPage), _translate("MainWindow", "info", None))
        self.left_tabs.setTabText(self.left_tabs.indexOf(self.gpsPage), _translate("MainWindow", "gps", None))
        self.label_3.setText(_translate("MainWindow", "blue", None))
        self.label.setText(_translate("MainWindow", "red", None))
        self.label_2.setText(_translate("MainWindow", "green", None))
        self.led_label_0.setText(_translate("MainWindow", "LED strip 0", None))
        self.label_8.setText(_translate("MainWindow", "blue", None))
        self.label_9.setText(_translate("MainWindow", "red", None))
        self.label_10.setText(_translate("MainWindow", "green", None))
        self.led_label_1.setText(_translate("MainWindow", "LED strip 1", None))
        self.label_11.setText(_translate("MainWindow", "blue", None))
        self.label_12.setText(_translate("MainWindow", "red", None))
        self.label_13.setText(_translate("MainWindow", "green", None))
        self.led_label_2.setText(_translate("MainWindow", "LED strip 2", None))
        self.led_label_3.setText(_translate("MainWindow", "LED strip 3", None))
        self.label_14.setText(_translate("MainWindow", "blue", None))
        self.label_15.setText(_translate("MainWindow", "red", None))
        self.label_16.setText(_translate("MainWindow", "green", None))
        self.pushButton_led_strip_update.setText(_translate("MainWindow", "update LED strips", None))
        self.checkBox_led_strip_update_continuous.setText(_translate("MainWindow", "continuous", None))
        self.left_tabs.setTabText(self.left_tabs.indexOf(self.ledPage), _translate("MainWindow", "led", None))
        self.label_parameter_10.setText(_translate("MainWindow", "parameter10", None))
        self.label_parameter_00.setText(_translate("MainWindow", "parameter0", None))
        self.label_parameter_15.setText(_translate("MainWindow", "parameter15", None))
        self.label_parameter_03.setText(_translate("MainWindow", "parameter3", None))
        self.label_parameter_01.setText(_translate("MainWindow", "parameter1", None))
        self.label_parameter_04.setText(_translate("MainWindow", "parameter4", None))
        self.label_parameter_11.setText(_translate("MainWindow", "parameter11", None))
        self.label_parameter_06.setText(_translate("MainWindow", "parameter6", None))
        self.label_parameter_07.setText(_translate("MainWindow", "parameter7", None))
        self.label_parameter_02.setText(_translate("MainWindow", "parameter2", None))
        self.label_parameter_05.setText(_translate("MainWindow", "parameter5", None))
        self.label_parameter_09.setText(_translate("MainWindow", "parameter9", None))
        self.label_parameter_08.setText(_translate("MainWindow", "parameter8", None))
        self.label_parameter_13.setText(_translate("MainWindow", "parameter13", None))
        self.label_parameter_14.setText(_translate("MainWindow", "parameter14", None))
        self.label_parameter_12.setText(_translate("MainWindow", "parameter12", None))
        self.label_parameter_16.setText(_translate("MainWindow", "parameter16", None))
        self.label_parameter_17.setText(_translate("MainWindow", "parameter17", None))
        self.checkBox_parameters_update_continuous.setText(_translate("MainWindow", "continuous", None))
        self.pushButton_parameters_update.setText(_translate("MainWindow", "update parameters", None))
        self.right_tabs.setTabText(self.right_tabs.indexOf(self.parameter_tab), _translate("MainWindow", "parameters", None))
        self.right_tabs.setTabText(self.right_tabs.indexOf(self.altitude_tab), _translate("MainWindow", "altitude", None))

from pyqtgraph import PlotWidget