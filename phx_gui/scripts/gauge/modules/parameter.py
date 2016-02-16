__author__ = 'satellite'


class ParameterTab:
    def __init__(self, ui_win):
        self.ui_win = ui_win
    
    def set_parameters_lcd(self, number, val=0):
        if number == 0:
            self.ui_win.lcdNumber_parameter_00.display(val)
        elif number == 1:
            self.ui_win.lcdNumber_parameter_01.display(val)
        elif number == 2:
            self.ui_win.lcdNumber_parameter_02.display(val)
        elif number == 3:
            self.ui_win.lcdNumber_parameter_03.display(val)
        elif number == 4:
            self.ui_win.lcdNumber_parameter_04.display(val)
        elif number == 5:
            self.ui_win.lcdNumber_parameter_05.display(val)
        elif number == 6:
            self.ui_win.lcdNumber_parameter_06.display(val)
        elif number == 7:
            self.ui_win.lcdNumber_parameter_07.display(val)
        elif number == 8:
            self.ui_win.lcdNumber_parameter_08.display(val)
        elif number == 9:
            self.ui_win.lcdNumber_parameter_09.display(val)
        elif number == 10:
            self.ui_win.lcdNumber_parameter_10.display(val)
        elif number == 11:
            self.ui_win.lcdNumber_parameter_11.display(val)
        elif number == 12:
            self.ui_win.lcdNumber_parameter_12.display(val)
        elif number == 13:
            self.ui_win.lcdNumber_parameter_13.display(val)
        elif number == 14:
            self.ui_win.lcdNumber_parameter_14.display(val)
        elif number == 15:
            self.ui_win.lcdNumber_parameter_15.display(val)
        elif number == 16:
            self.ui_win.lcdNumber_parameter_16.display(val)
        elif number == 17:
            self.ui_win.lcdNumber_parameter_17.display(val)
        else:
            print ' -> set_parameters_lcd requested number', number, 'not available'
    
    def set_parameters_slider_limits(self, number, min=10, max=2000):
        if number == 0:
            self.ui_win.horizontalSlider_parameter_00.setRange(min, max)
        elif number == 1:
            self.ui_win.horizontalSlider_parameter_01.setRange(min, max)
        elif number == 2:
            self.ui_win.horizontalSlider_parameter_02.setRange(min, max)
        elif number == 3:
            self.ui_win.horizontalSlider_parameter_03.setRange(min, max)
        elif number == 4:
            self.ui_win.horizontalSlider_parameter_04.setRange(min, max)
        elif number == 5:
            self.ui_win.horizontalSlider_parameter_05.setRange(min, max)
        elif number == 6:
            self.ui_win.horizontalSlider_parameter_06.setRange(min, max)
        elif number == 7:
            self.ui_win.horizontalSlider_parameter_07.setRange(min, max)
        elif number == 8:
            self.ui_win.horizontalSlider_parameter_08.setRange(min, max)
        elif number == 9:
            self.ui_win.horizontalSlider_parameter_09.setRange(min, max)
        elif number == 10:
            self.ui_win.horizontalSlider_parameter_10.setRange(min, max)
        elif number == 11:
            self.ui_win.horizontalSlider_parameter_11.setRange(min, max)
        elif number == 12:
            self.ui_win.horizontalSlider_parameter_12.setRange(min, max)
        elif number == 13:
            self.ui_win.horizontalSlider_parameter_13.setRange(min, max)
        elif number == 14:
            self.ui_win.horizontalSlider_parameter_14.setRange(min, max)
        elif number == 15:
            self.ui_win.horizontalSlider_parameter_15.setRange(min, max)
        elif number == 16:
            self.ui_win.horizontalSlider_parameter_16.setRange(min, max)
        elif number == 17:
            self.ui_win.horizontalSlider_parameter_17.setRange(min, max)
        else:
            print ' -> set_parameters_slider_limits requested number', number, 'not available'
    
    def set_parameters_slider(self, number, val, lcd_linked=True, update_slider=True):
        if update_slider is False:
            pass
        elif number == 0:
            self.ui_win.horizontalSlider_parameter_00.setValue(val)
        elif number == 1:
            self.ui_win.horizontalSlider_parameter_01.setValue(val)
        elif number == 2:
            self.ui_win.horizontalSlider_parameter_02.setValue(val)
        elif number == 3:
            self.ui_win.horizontalSlider_parameter_03.setValue(val)
        elif number == 4:
            self.ui_win.horizontalSlider_parameter_04.setValue(val)
        elif number == 5:
            self.ui_win.horizontalSlider_parameter_05.setValue(val)
        elif number == 6:
            self.ui_win.horizontalSlider_parameter_06.setValue(val)
        elif number == 7:
            self.ui_win.horizontalSlider_parameter_07.setValue(val)
        elif number == 8:
            self.ui_win.horizontalSlider_parameter_08.setValue(val)
        elif number == 9:
            self.ui_win.horizontalSlider_parameter_09.setValue(val)
        elif number == 10:
            self.ui_win.horizontalSlider_parameter_10.setValue(val)
        elif number == 11:
            self.ui_win.horizontalSlider_parameter_11.setValue(val)
        elif number == 12:
            self.ui_win.horizontalSlider_parameter_12.setValue(val)
        elif number == 13:
            self.ui_win.horizontalSlider_parameter_13.setValue(val)
        elif number == 14:
            self.ui_win.horizontalSlider_parameter_14.setValue(val)
        elif number == 15:
            self.ui_win.horizontalSlider_parameter_15.setValue(val)
        elif number == 16:
            self.ui_win.horizontalSlider_parameter_16.setValue(val)
        elif number == 17:
            self.ui_win.horizontalSlider_parameter_17.setValue(val)
        else:
            print ' -> set_parameters_slider requested number', number, 'not available'
        if lcd_linked:
            self.set_parameters_lcd(number, val)
    
    def get_parameters_slider(self, number):
        if number == 0:
            return self.ui_win.horizontalSlider_parameter_00.value()
        elif number == 1:
            return self.ui_win.horizontalSlider_parameter_01.value()
        elif number == 2:
            return self.ui_win.horizontalSlider_parameter_02.value()
        elif number == 3:
            return self.ui_win.horizontalSlider_parameter_03.value()
        elif number == 4:
            return self.ui_win.horizontalSlider_parameter_04.value()
        elif number == 5:
            return self.ui_win.horizontalSlider_parameter_05.value()
        elif number == 6:
            return self.ui_win.horizontalSlider_parameter_06.value()
        elif number == 7:
            return self.ui_win.horizontalSlider_parameter_07.value()
        elif number == 8:
            return self.ui_win.horizontalSlider_parameter_08.value()
        elif number == 9:
            return self.ui_win.horizontalSlider_parameter_09.value()
        elif number == 10:
            return self.ui_win.horizontalSlider_parameter_10.value()
        elif number == 11:
            return self.ui_win.horizontalSlider_parameter_11.value()
        elif number == 12:
            return self.ui_win.horizontalSlider_parameter_12.value()
        elif number == 13:
            return self.ui_win.horizontalSlider_parameter_13.value()
        elif number == 14:
            return self.ui_win.horizontalSlider_parameter_14.value()
        elif number == 15:
            return self.ui_win.horizontalSlider_parameter_15.value()
        elif number == 16:
            return self.ui_win.horizontalSlider_parameter_16.value()
        elif number == 17:
            return self.ui_win.horizontalSlider_parameter_17.value()
        else:
            print ' -> get_parameters_slider requested number', number, 'not available'
            return False