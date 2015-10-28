
from PyQt4 import uic, QtCore, QtGui
import layouts
import modules
import gauge_ros


"""
    build QT application
"""
app = QtGui.QApplication([])
win = layouts.MainWindow()
ui_win = layouts.Ui_MainWindow()
ui_win.setupUi(win)

win.setWindowTitle('gauge GUI - made for ROS')
ui_win.statusbar.showMessage("starting up...")

# init gps tab
gps_tab = modules.gps.GPStab(graphicsView_gps=ui_win.graphicsView_gps,
                             text_boxes=(ui_win.gps_position_home_TextBrowser,
                                         ui_win.gps_position_way_point_TextBrowser,
                                         ui_win.gps_position_current_TextBrowser),
                             mouse_click_callback=None,
                             mouse_move_callback=ui_win.statusbar,
                             way_point_list=ui_win.way_points_tableWidget)
# gps_tab.update_gps_map(use_map='/home/satellite/ros_catkin_ws/src/phx_quadrocopter_ros/phx_gui/scripts/maps/map3.npz')
gps_tab.update_gps_map(use_map=3)

# init led tab
led_tab = modules.led.LEDtab(strip0=(ui_win.horizontalSlider_led_0_r,
                                     ui_win.horizontalSlider_led_0_g,
                                     ui_win.horizontalSlider_led_0_b),
                             strip1=(ui_win.horizontalSlider_led_1_r,
                                     ui_win.horizontalSlider_led_1_g,
                                     ui_win.horizontalSlider_led_1_b),
                             strip2=(ui_win.horizontalSlider_led_2_r,
                                     ui_win.horizontalSlider_led_2_g,
                                     ui_win.horizontalSlider_led_2_b),
                             strip3=(ui_win.horizontalSlider_led_3_r,
                                     ui_win.horizontalSlider_led_3_g,
                                     ui_win.horizontalSlider_led_3_b),
                             ros_publish_function=None)
QtCore.QObject.connect(ui_win.pushButton_led_strip_update, QtCore.SIGNAL('clicked()'), led_tab.send_all_strips)
QtCore.QObject.connect(ui_win.led_comboBox_mode_selection, QtCore.SIGNAL('currentIndexChanged(QString)'), led_tab.set_mode)

# init pid tab
pid_tab = modules.pid.PIDtab(pid_config_storage_path='',
                             ui_win=ui_win,
                             ros_publish_function=None)
QtCore.QObject.connect(ui_win.pushButton_pid_set, QtCore.SIGNAL('clicked()'), pid_tab.send_pid_values)
QtCore.QObject.connect(ui_win.spinBox_pid_config_storage, QtCore.SIGNAL('valueChanged(int)'), pid_tab.pid_config_storage_comment_update)
QtCore.QObject.connect(ui_win.pushButton_pid_config_storage_save, QtCore.SIGNAL('clicked()'), pid_tab.pid_config_storage_comment_save)
QtCore.QObject.connect(ui_win.pushButton_pid_config_storage_load, QtCore.SIGNAL('clicked()'), pid_tab.pid_config_storage_comment_load)

# init altitude tab
altitude_tab = modules.altitude.AltitudeTab(graphicsView_altitude=ui_win.graphicsView_altitude)

# init video tab
video_tab = modules.video.VIDEOtab(graphicsView_video=ui_win.graphicsView_video,
                                   video_active=ui_win.checkBox_video_active.isChecked,
                                   video_reset=ui_win.checkBox_video_reset_ranges.isChecked)

# init rc fc tab NAZE
rc_fc_tab = modules.rc.RCtab(graphicsView_rc=ui_win.graphicsView_rc_fc,
                             sampling_length=1000,
                             sliders=(ui_win.remote_slider_rc_fc_pitch,
                                      ui_win.remote_slider_rc_fc_roll,
                                      ui_win.remote_slider_rc_fc_yaw,
                                      ui_win.remote_slider_rc_fc_throttle,
                                      ui_win.remote_slider_rc_fc_aux1,
                                      ui_win.remote_slider_rc_fc_aux2,
                                      ui_win.remote_slider_rc_fc_aux3,
                                      ui_win.remote_slider_rc_fc_aux4)
                             )

# init rc fc tab MARVIC
rc_marvic_tab = modules.rc.RCtab(graphicsView_rc=ui_win.graphicsView_rc_marvic,
                                 sampling_length=1000,
                                 sliders=(ui_win.remote_slider_rc_marvic_pitch,
                                          ui_win.remote_slider_rc_marvic_roll,
                                          ui_win.remote_slider_rc_marvic_yaw,
                                          ui_win.remote_slider_rc_marvic_throttle,
                                          ui_win.remote_slider_rc_marvic_aux1,
                                          ui_win.remote_slider_rc_marvic_aux2,
                                          ui_win.remote_slider_rc_marvic_aux3,
                                          ui_win.remote_slider_rc_marvic_aux4)
                                 )

# init parameter
parameter_tab = modules.parameter.ParameterTab(ui_win=ui_win)
for i in range(0, 18):
    parameter_tab.set_parameters_slider_limits(i, 300, 2450)

# init ros node
ros_node = gauge_ros.ROSgauge(gps_tab=gps_tab,
                              led_tab=led_tab,
                              pid_tab=pid_tab,
                              altitude_tab=altitude_tab,
                              video_tab=video_tab,
                              rc_fc_tab=rc_fc_tab,
                              rc_marvic_tab=rc_marvic_tab,
                              parameter_tab=parameter_tab
                              )
gps_tab.mouse_click_callback = [None,
                                ros_node.publish_gps_add_way_point,
                                ros_node.publish_gps_way_point]
gps_tab.way_point_tab.publish_way_point_remove = ros_node.publish_gps_remove_way_point
led_tab.ros_publish_function = ros_node.publish_led_strip
pid_tab.ros_publish_function = ros_node.publish_pid

QtCore.QObject.connect(ui_win.pushButton_parameters_update, QtCore.SIGNAL('clicked()'), ros_node.publish_servos)
QtCore.QObject.connect(ui_win.gps_comboBox_wp_controller, QtCore.SIGNAL('activated(int)'), ros_node.publish_management_gps_way_point_controller)
QtCore.QObject.connect(ui_win.way_points_pushButton_remove, QtCore.SIGNAL('clicked()'), gps_tab.way_point_tab.way_point_remove)


def mainloop():
    # gps
    gps_tab.update_gps_plot()
    if gps_tab.way_point_tab:
        gps_tab.way_point_tab.update_list()

    # video
    ros_node.optimize_ros_video_subscription(active=ui_win.checkBox_video_active.isChecked())
    video_tab.update_video()

    # led
    if ui_win.checkBox_led_strip_update_continuous.isChecked():
        led_tab.send_all_strips()

    # rc
    rc_fc_tab.update_rc(ui_win.remote_checkBox_rc_fc_plot.isChecked())
    rc_marvic_tab.update_rc(ui_win.remote_checkBox_rc_marvic_plot.isChecked())

    # altitude
    altitude_tab.update_altitude_plot()

    # parameter
    if ui_win.checkBox_parameters_update_continuous.isChecked():
        ros_node.publish_servos()

win.show()
# QTimer
timer = QtCore.QTimer()
timer.timeout.connect(mainloop)
timer.start(100)
app.exec_()

timer.stop()

ros_node.shutdown_ros()
