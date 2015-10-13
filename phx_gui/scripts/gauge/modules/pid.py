import numpy as np
import os

from phx_arduino_uart_bridge.msg import PID
from phx_arduino_uart_bridge.msg import PID_cleanflight


class PIDtab:
    def __init__(self, pid_config_storage_path, ui_win, ros_publish_function=None):
        self.ui_win = ui_win
        self.ros_publish_function = ros_publish_function
        self.pid_config_storage_comments_file = pid_config_storage_path + 'pid_config_storage_comments.txt'
        self.pid_config_storage_file = pid_config_storage_path + 'pid_config_storage.txt'

        # load from file if there is any file!
        if os.path.isfile(self.pid_config_storage_comments_file):
            read_file = open(self.pid_config_storage_comments_file, 'r')
            self.pid_config_storage_comments = [i.strip() for i in read_file.readlines()]
            read_file.close()
            self.pid_config_storage = np.loadtxt(self.pid_config_storage_file, dtype=np.uint8)
        else:
            self.pid_config_storage_comments = ['empty'] * 10
            self.pid_config_storage = np.zeros((10, 30))

    def save_pid_config(self):
        write_file = open(self.pid_config_storage_comments_file, 'w')
        for comment in self.pid_config_storage_comments:
            write_file.write(comment)
            write_file.write('\n')
        write_file.close()
        write_file = open(self.pid_config_storage_file, 'w')
        for line in self.pid_config_storage:
            for integer in line:
                write_file.write(str(integer))
                write_file.write('\t')
            write_file.write('\n')
        write_file.close()

    def pid_config_storage_comment_update(self, index):
        self.ui_win.pid_lineEdit_config_storage.setText(self.pid_config_storage_comments[index])
    
    def pid_config_storage_comment_save(self):
        index = self.ui_win.spinBox_pid_config_storage.value()
        self.pid_config_storage[index, 0] = self.ui_win.pid_spinBox_roll_p.value()
        self.pid_config_storage[index, 1] = self.ui_win.pid_spinBox_roll_i.value()
        self.pid_config_storage[index, 2] = self.ui_win.pid_spinBox_roll_d.value()
        self.pid_config_storage[index, 3] = self.ui_win.pid_spinBox_pitch_p.value()
        self.pid_config_storage[index, 4] = self.ui_win.pid_spinBox_pitch_i.value()
        self.pid_config_storage[index, 5] = self.ui_win.pid_spinBox_pitch_d.value()
        self.pid_config_storage[index, 6] = self.ui_win.pid_spinBox_yaw_p.value()
        self.pid_config_storage[index, 7] = self.ui_win.pid_spinBox_yaw_i.value()
        self.pid_config_storage[index, 8] = self.ui_win.pid_spinBox_yaw_d.value()
        self.pid_config_storage[index, 9] = self.ui_win.pid_spinBox_alt_p.value()
        self.pid_config_storage[index, 10] = self.ui_win.pid_spinBox_alt_i.value()
        self.pid_config_storage[index, 11] = self.ui_win.pid_spinBox_alt_d.value()
        self.pid_config_storage[index, 12] = self.ui_win.pid_spinBox_vel_p.value()
        self.pid_config_storage[index, 13] = self.ui_win.pid_spinBox_vel_i.value()
        self.pid_config_storage[index, 14] = self.ui_win.pid_spinBox_vel_d.value()
        self.pid_config_storage[index, 15] = self.ui_win.pid_spinBox_pos_p.value()
        self.pid_config_storage[index, 16] = self.ui_win.pid_spinBox_pos_i.value()
        self.pid_config_storage[index, 17] = self.ui_win.pid_spinBox_pos_d.value()
        self.pid_config_storage[index, 18] = self.ui_win.pid_spinBox_posrate_p.value()
        self.pid_config_storage[index, 19] = self.ui_win.pid_spinBox_posrate_i.value()
        self.pid_config_storage[index, 20] = self.ui_win.pid_spinBox_posrate_d.value()
        self.pid_config_storage[index, 21] = self.ui_win.pid_spinBox_navrate_p.value()
        self.pid_config_storage[index, 22] = self.ui_win.pid_spinBox_navrate_i.value()
        self.pid_config_storage[index, 23] = self.ui_win.pid_spinBox_navrate_d.value()
        self.pid_config_storage[index, 24] = self.ui_win.pid_spinBox_level_p.value()
        self.pid_config_storage[index, 25] = self.ui_win.pid_spinBox_level_i.value()
        self.pid_config_storage[index, 26] = self.ui_win.pid_spinBox_level_d.value()
        self.pid_config_storage[index, 27] = self.ui_win.pid_spinBox_mag_p.value()
        self.pid_config_storage[index, 28] = self.ui_win.pid_spinBox_mag_i.value()
        self.pid_config_storage[index, 29] = self.ui_win.pid_spinBox_mag_d.value()

        self.pid_config_storage_comments[index] = str(self.ui_win.pid_lineEdit_config_storage.text())
        self.save_pid_config()
    
    def pid_config_storage_comment_load(self):
        index = self.ui_win.spinBox_pid_config_storage.value()
        self.ui_win.pid_spinBox_roll_p.setValue(self.pid_config_storage[index, 0])
        self.ui_win.pid_spinBox_roll_i.setValue(self.pid_config_storage[index, 1])
        self.ui_win.pid_spinBox_roll_d.setValue(self.pid_config_storage[index, 2])
        self.ui_win.pid_spinBox_pitch_p.setValue(self.pid_config_storage[index, 3])
        self.ui_win.pid_spinBox_pitch_i.setValue(self.pid_config_storage[index, 4])
        self.ui_win.pid_spinBox_pitch_d.setValue(self.pid_config_storage[index, 5])
        self.ui_win.pid_spinBox_yaw_p.setValue(self.pid_config_storage[index, 6])
        self.ui_win.pid_spinBox_yaw_i.setValue(self.pid_config_storage[index, 7])
        self.ui_win.pid_spinBox_yaw_d.setValue(self.pid_config_storage[index, 8])
        self.ui_win.pid_spinBox_alt_p.setValue(self.pid_config_storage[index, 9])
        self.ui_win.pid_spinBox_alt_i.setValue(self.pid_config_storage[index, 10])
        self.ui_win.pid_spinBox_alt_d.setValue(self.pid_config_storage[index, 11])
        self.ui_win.pid_spinBox_vel_p.setValue(self.pid_config_storage[index, 12])
        self.ui_win.pid_spinBox_vel_i.setValue(self.pid_config_storage[index, 13])
        self.ui_win.pid_spinBox_vel_d.setValue(self.pid_config_storage[index, 14])
        self.ui_win.pid_spinBox_pos_p.setValue(self.pid_config_storage[index, 15])
        self.ui_win.pid_spinBox_pos_i.setValue(self.pid_config_storage[index, 16])
        self.ui_win.pid_spinBox_pos_d.setValue(self.pid_config_storage[index, 17])
        self.ui_win.pid_spinBox_posrate_p.setValue(self.pid_config_storage[index, 18])
        self.ui_win.pid_spinBox_posrate_i.setValue(self.pid_config_storage[index, 19])
        self.ui_win.pid_spinBox_posrate_d.setValue(self.pid_config_storage[index, 20])
        self.ui_win.pid_spinBox_navrate_p.setValue(self.pid_config_storage[index, 21])
        self.ui_win.pid_spinBox_navrate_i.setValue(self.pid_config_storage[index, 22])
        self.ui_win.pid_spinBox_navrate_d.setValue(self.pid_config_storage[index, 23])
        self.ui_win.pid_spinBox_level_p.setValue(self.pid_config_storage[index, 24])
        self.ui_win.pid_spinBox_level_i.setValue(self.pid_config_storage[index, 25])
        self.ui_win.pid_spinBox_level_d.setValue(self.pid_config_storage[index, 26])
        self.ui_win.pid_spinBox_mag_p.setValue(self.pid_config_storage[index, 27])
        self.ui_win.pid_spinBox_mag_i.setValue(self.pid_config_storage[index, 28])
        self.ui_win.pid_spinBox_mag_d.setValue(self.pid_config_storage[index, 29])
    
    def callback_fc_pid_cleanflight(self, cur_pid_settings):
        # this function is located here for simplicity writing the code!!!
        if self.ui_win.checkBox_pid_update.isChecked():
            self.ui_win.checkBox_pid_update.setChecked(False)
            self.ui_win.pid_spinBox_roll_p.setValue(cur_pid_settings.roll.p)
            self.ui_win.pid_spinBox_roll_i.setValue(cur_pid_settings.roll.i)
            self.ui_win.pid_spinBox_roll_d.setValue(cur_pid_settings.roll.d)
            self.ui_win.pid_spinBox_pitch_p.setValue(cur_pid_settings.pitch.p)
            self.ui_win.pid_spinBox_pitch_i.setValue(cur_pid_settings.pitch.i)
            self.ui_win.pid_spinBox_pitch_d.setValue(cur_pid_settings.pitch.d)
            self.ui_win.pid_spinBox_yaw_p.setValue(cur_pid_settings.yaw.p)
            self.ui_win.pid_spinBox_yaw_i.setValue(cur_pid_settings.yaw.i)
            self.ui_win.pid_spinBox_yaw_d.setValue(cur_pid_settings.yaw.d)
            self.ui_win.pid_spinBox_alt_p.setValue(cur_pid_settings.alt.p)
            self.ui_win.pid_spinBox_alt_i.setValue(cur_pid_settings.alt.i)
            self.ui_win.pid_spinBox_alt_d.setValue(cur_pid_settings.alt.d)
            self.ui_win.pid_spinBox_alt_p.setValue(cur_pid_settings.alt.p)
            self.ui_win.pid_spinBox_alt_i.setValue(cur_pid_settings.alt.i)
            self.ui_win.pid_spinBox_alt_d.setValue(cur_pid_settings.alt.d)
            self.ui_win.pid_spinBox_pos_p.setValue(cur_pid_settings.pos.p)
            self.ui_win.pid_spinBox_pos_i.setValue(cur_pid_settings.pos.i)
            self.ui_win.pid_spinBox_pos_d.setValue(cur_pid_settings.pos.d)
            self.ui_win.pid_spinBox_posrate_p.setValue(cur_pid_settings.posrate.p)
            self.ui_win.pid_spinBox_posrate_i.setValue(cur_pid_settings.posrate.i)
            self.ui_win.pid_spinBox_posrate_d.setValue(cur_pid_settings.posrate.d)
            self.ui_win.pid_spinBox_navrate_p.setValue(cur_pid_settings.navrate.p)
            self.ui_win.pid_spinBox_navrate_i.setValue(cur_pid_settings.navrate.i)
            self.ui_win.pid_spinBox_navrate_d.setValue(cur_pid_settings.navrate.d)
            self.ui_win.pid_spinBox_level_p.setValue(cur_pid_settings.level.p)
            self.ui_win.pid_spinBox_level_i.setValue(cur_pid_settings.level.i)
            self.ui_win.pid_spinBox_level_d.setValue(cur_pid_settings.level.d)
            self.ui_win.pid_spinBox_mag_p.setValue(cur_pid_settings.mag.p)
            self.ui_win.pid_spinBox_mag_i.setValue(cur_pid_settings.mag.i)
            self.ui_win.pid_spinBox_mag_d.setValue(cur_pid_settings.mag.d)
        else:
            pass
    
    def send_pid_values(self):
        if self.ui_win.checkBox_pid_active.isChecked() and self.ros_publish_function:
            pid_msg = PID_cleanflight()
            pid_msg.roll.p = self.ui_win.pid_spinBox_roll_p.value()
            pid_msg.roll.i = self.ui_win.pid_spinBox_roll_i.value()
            pid_msg.roll.d = self.ui_win.pid_spinBox_roll_d.value()
            pid_msg.pitch.p = self.ui_win.pid_spinBox_pitch_p.value()
            pid_msg.pitch.i = self.ui_win.pid_spinBox_pitch_i.value()
            pid_msg.pitch.d = self.ui_win.pid_spinBox_pitch_d.value()
            pid_msg.yaw.p = self.ui_win.pid_spinBox_yaw_p.value()
            pid_msg.yaw.i = self.ui_win.pid_spinBox_yaw_i.value()
            pid_msg.yaw.d = self.ui_win.pid_spinBox_yaw_d.value()
            pid_msg.alt.p = self.ui_win.pid_spinBox_alt_p.value()
            pid_msg.alt.i = self.ui_win.pid_spinBox_alt_i.value()
            pid_msg.alt.d = self.ui_win.pid_spinBox_alt_d.value()
            pid_msg.vel.p = self.ui_win.pid_spinBox_vel_p.value()
            pid_msg.vel.i = self.ui_win.pid_spinBox_vel_i.value()
            pid_msg.vel.d = self.ui_win.pid_spinBox_vel_d.value()
            pid_msg.pos.p = self.ui_win.pid_spinBox_pos_p.value()
            pid_msg.pos.i = self.ui_win.pid_spinBox_pos_i.value()
            pid_msg.pos.d = self.ui_win.pid_spinBox_pos_d.value()
            pid_msg.posrate.p = self.ui_win.pid_spinBox_posrate_p.value()
            pid_msg.posrate.i = self.ui_win.pid_spinBox_posrate_i.value()
            pid_msg.posrate.d = self.ui_win.pid_spinBox_posrate_d.value()
            pid_msg.navrate.p = self.ui_win.pid_spinBox_navrate_p.value()
            pid_msg.navrate.i = self.ui_win.pid_spinBox_navrate_i.value()
            pid_msg.navrate.d = self.ui_win.pid_spinBox_navrate_d.value()
            pid_msg.level.p = self.ui_win.pid_spinBox_level_p.value()
            pid_msg.level.i = self.ui_win.pid_spinBox_level_i.value()
            pid_msg.level.d = self.ui_win.pid_spinBox_level_d.value()
            pid_msg.mag.p = self.ui_win.pid_spinBox_mag_p.value()
            pid_msg.mag.i = self.ui_win.pid_spinBox_mag_i.value()
            pid_msg.mag.d = self.ui_win.pid_spinBox_mag_d.value()
            self.ros_publish_function(pid_msg)
        else:
            print 'not allowed'
    