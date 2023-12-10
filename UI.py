import dearpygui.dearpygui as dpg
import threading
import math
import serial
import numpy as np
import time
import os

SAMPLING_RATE = 1000 # sps
BUZZER_ALERT_TIME = 0.5 # seconds

class ArduinoController:

    def __init__(self):
        self.ser = serial.Serial('COM13', 115200)
        self.__acc_buffer = [[],[],[]]
        self.data_x = np.arange(0,len(self.__acc_buffer),1)
        self.gyroX = np.array(np.zeros(len(self.__acc_buffer)))
        self.gyroY = np.array(np.zeros(len(self.__acc_buffer)))
        self.gyroZ = np.array(np.zeros(len(self.__acc_buffer)))
        self.resolution = 2 #Bytes (At least 2)
        self.pitch = 0
        self.roll = 0
        self.__angle_buffer =[[],[]]
        self.__is_training = False
        self.start_training_time = 0
        self.__trainee_name = ""
        self.__trainer_name = ""
        self.__pitch_error = 0
        self.__roll_error = 0
        self.__pitch_calibration = 30
        self.__roll_calibration = 0
        self.__pitch_deadzone = 30
        self.__roll_deadzone = 30
        self.__buzzer_start_time = 0
        
    def create_package_object(self, header_type, payload):
        # convert package_object to bytearray
        package_bytes = bytearray(header_type.value + payload)
        print(package_bytes)
        return package_bytes
        
    def render(self):
        dpg.configure_app(init_file="UI_config.ini")
        y_acc_min = -1
        y_acc_max = 1       
        y_angle_min = -180
        y_angle_max = 180
        # Plot gyro value
        with dpg.window(label="Acceleration", height=1080, width=800):
            with dpg.plot(label="Acceleration X-axis", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_accX = dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_accX", no_tick_labels=False)
                y_axis_accX = dpg.add_plot_axis(dpg.mvYAxis, label="Acceleration (g-force)", tag="y_axis_accX")
                dpg.set_axis_limits(y_axis_accX, y_acc_min, y_acc_max)
                dpg.add_line_series(self.data_x, self.gyroX, label="Acceleration X-axis", parent="y_axis_accX", tag="acc_plotX")
            
            with dpg.plot(label="Acceleration Y-axis", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_accY = dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_accY", no_tick_labels=False)
                y_axis_accY = dpg.add_plot_axis(dpg.mvYAxis, label="Acceleration (g-force)", tag="y_axis_accY")
                dpg.set_axis_limits(y_axis_accY, y_acc_min, y_acc_max)
                dpg.add_line_series(self.data_x, self.gyroY, label="Acceleration Y-axis", parent="y_axis_accY", tag="acc_plotY")
            
            with dpg.plot(label="Acceleration Z-axis", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_accZ = dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_accZ", no_tick_labels=False)
                y_axis_accZ = dpg.add_plot_axis(dpg.mvYAxis, label="Acceleration (g-force)", tag="y_axis_accZ")
                dpg.set_axis_limits(y_axis_accZ, y_acc_min, y_acc_max)
                dpg.add_line_series(self.data_x, self.gyroZ, label="Acceleration Z-axis", parent="y_axis_accZ", tag="acc_plotZ")
        
        with dpg.window(label="Angular Rotation of Syringe", height=800, width=800):
            with dpg.plot(label="Pitch", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_accX = dpg.add_plot_axis(dpg.mvXAxis, label="angle", tag="x_axis_pitch", no_tick_labels=False)
                y_axis_accX = dpg.add_plot_axis(dpg.mvYAxis, label="Pitch Angle (Degree)", tag="y_axis_pitch")
                dpg.set_axis_limits(y_axis_accX, y_angle_min, y_angle_max)
                dpg.add_line_series(self.data_x, self.gyroX, label="pitch", parent="y_axis_pitch", tag="pitch_plotX")
            with dpg.group(horizontal=True):
                dpg.add_text("Current pitch:")
                dpg.add_text(tag="current_pitch", default_value="0")
                dpg.add_text("Degree")
            with dpg.group(horizontal=True):
                dpg.add_text("Current pitch error:")
                dpg.add_text(tag="current_pitch_error", default_value="0")
                dpg.add_text("Degree")
                
            with dpg.plot(label="Roll", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_accY = dpg.add_plot_axis(dpg.mvXAxis, label="angle", tag="x_axis_roll", no_tick_labels=False)
                y_axis_accY = dpg.add_plot_axis(dpg.mvYAxis, label="Roll Angle (degree)", tag="y_axis_roll")
                dpg.set_axis_limits(y_axis_accY, y_angle_min, y_angle_max)
                dpg.add_line_series(self.data_x, self.gyroY, label="roll", parent="y_axis_roll", tag="roll_plotY")
            with dpg.group(horizontal=True):
                dpg.add_text("Current roll:")
                dpg.add_text(tag="current_roll", default_value="0")
                dpg.add_text("Degree")
            with dpg.group(horizontal=True):
                dpg.add_text("Current roll error:")
                dpg.add_text(tag="current_roll_error", default_value="0")
                dpg.add_text("Degree")
                
        with dpg.window(label="Recording Monitor", height=400, width=300):
            dpg.add_text("[Calibration]")
            dpg.add_button(label="Calibrate!", callback=self.measure, height=30, width=100)
            with dpg.group(horizontal=True):
                dpg.add_text("Pitch Calibration angle:")
                dpg.add_input_int(default_value=30, tag="pitch_calibration", width=-1)
            with dpg.group(horizontal=True):
                dpg.add_text("Roll Calibration angle:")
                dpg.add_input_int(default_value=0, tag="roll_calibration", width=-1)
            with dpg.group(horizontal=True):
                dpg.add_text("Pitch Deadzone angle:")
                dpg.add_input_int(default_value=30, tag="pitch_deadzone", width=-1)
            with dpg.group(horizontal=True):
                dpg.add_text("Roll Deadzone angle:")
                dpg.add_input_int(default_value=30, tag="roll_deadzone", width=-1)
            with dpg.group(horizontal=True):
                dpg.add_text("Pitch:")
                dpg.add_text(tag="pitch_value", default_value="0")
            with dpg.group(horizontal=True):
                dpg.add_text("Pitch Error Angle:")
                dpg.add_text(tag="pitch_error", default_value="0")
            with dpg.group(horizontal=True):
                dpg.add_text("Roll:")
                dpg.add_text(tag="roll_value", default_value="0")
            with dpg.group(horizontal=True):
                dpg.add_text("Roll Error Angle:")
                dpg.add_text(tag="roll_error", default_value="0")
            dpg.add_text("========================================================================")
            
            dpg.add_text("[Record data]")
            with dpg.group(horizontal=True):
                dpg.add_text("Trainee name:")
                dpg.add_input_text(default_value="Trainee", tag="trainee_name", width=-1)
            with dpg.group(horizontal=True):
                dpg.add_text("Trainer name:")
                dpg.add_input_text(default_value="Trainer", tag="trainer_name", width=-1)
            dpg.add_button(label="Record!", callback=self.toggle_training, tag="training_toggle_btn", height=30, width=100)
            dpg.add_text("Not Recording", tag="training_status")
            with dpg.group(horizontal=True):
                dpg.add_text("Current Operating time:")
                dpg.add_text(tag="current_training_time", default_value="0")
            dpg.add_text("========================================================================")
            
            dpg.add_text("[Record Summary]")
            with dpg.group(horizontal=True):
                dpg.add_text("Trainee name:")
                dpg.add_text(tag="trainee_name_summary", default_value="Trainee")
            with dpg.group(horizontal=True):
                dpg.add_text("Trainer name:")
                dpg.add_text(tag="trainer_name_summary", default_value="Trainer")
            with dpg.group(horizontal=True):
                dpg.add_text("Time operated:")
                dpg.add_text(tag="training_time", default_value="0")
            
    def update(self):
        try:
            inputdata = self.ser.read(6 * self.resolution)
            for i in range(len(self.__acc_buffer)):
                acc_data = int.from_bytes(inputdata[i * self.resolution : (i + 1) * self.resolution], byteorder='little', signed=True)
                self.__acc_buffer[i].append(acc_data/16384)
                if len(self.__acc_buffer[i]) >= 3000:
                    self.__acc_buffer[i].pop(0)
            
            # Calculate pitch and roll
            self.pitch = -math.atan2(self.__acc_buffer[0][-1], math.sqrt((self.__acc_buffer[1][-1])**2+self.__acc_buffer[2][-1]**2)) * 180 / math.pi
            self.roll = math.atan2(self.__acc_buffer[1][-1], math.sqrt((self.__acc_buffer[0][-1])**2+self.__acc_buffer[2][-1]**2)) * 180 / math.pi
            self.__angle_buffer[0].append(self.pitch)
            self.__angle_buffer[1].append(self.roll)
            if len(self.__angle_buffer[0]) >= 3000:
                self.__angle_buffer[0].pop(0)
                self.__angle_buffer[1].pop(0)
                
            # Write buzzer command "1" to Arduino if the pitch or roll is out of range
            if abs(self.pitch - self.__pitch_calibration) > self.__pitch_deadzone or abs(self.roll - self.__roll_calibration) > self.__roll_deadzone:
                # print("Buzzer alert: Pitch or roll is out of range")
                if time.time() - self.__buzzer_start_time > BUZZER_ALERT_TIME:
                    self.__buzzer_start_time = time.time()
                    self.ser.write(bytearray(b'\x01'))
            else:
                pass
                # print("No alert")
            
            # Plot gyro value
            self.data_x = np.arange(0, len(self.__acc_buffer[0]), 1)
            dpg.set_value("acc_plotX", [self.data_x, self.__acc_buffer[0]])
            dpg.set_value("acc_plotY", [self.data_x, self.__acc_buffer[1]])
            dpg.set_value("acc_plotZ", [self.data_x, self.__acc_buffer[2]])
            dpg.set_value("pitch_plotX", [self.data_x, self.__angle_buffer[0]])
            dpg.set_value("roll_plotY", [self.data_x, self.__angle_buffer[1]])
            dpg.fit_axis_data("x_axis_accX")
            dpg.fit_axis_data("y_axis_accX")
            dpg.fit_axis_data("x_axis_accY")
            dpg.fit_axis_data("y_axis_accY")
            dpg.fit_axis_data("x_axis_accZ")
            dpg.fit_axis_data("y_axis_accZ")
            dpg.fit_axis_data("x_axis_pitch")
            dpg.fit_axis_data("y_axis_pitch")
            dpg.fit_axis_data("x_axis_roll")
            dpg.fit_axis_data("y_axis_roll")
            
            self.__pitch_error = self.__pitch_calibration - self.pitch
            self.__roll_error = self.__roll_calibration - self.roll
            
            dpg.set_value("current_pitch", round(self.pitch, 2))
            dpg.set_value("current_roll", round(self.roll, 2))
            dpg.set_value("current_pitch_error", round(self.__pitch_error, 2))
            dpg.set_value("current_roll_error", round(self.__roll_error, 2))
            
            if self.__is_training:
                dpg.set_value("current_training_time", round((time.time() - self.start_training_time), 6))
            
        
        except Exception as e:
            print(e)
    
    def measure(self):
        dpg.set_value("pitch_value", self.pitch)
        dpg.set_value("roll_value", self.roll)
        dpg.set_value("pitch_error", round(self.__pitch_error, 2))
        dpg.set_value("roll_error", round(self.__roll_error, 2))
        self.__pitch_calibration = dpg.get_value("pitch_calibration")
        self.__roll_calibration = dpg.get_value("roll_calibration")
        self.__pitch_deadzone = dpg.get_value("pitch_deadzone")
        self.__roll_deadzone = dpg.get_value("roll_deadzone")
        print("Data is measured")
    
    def toggle_training(self, sender):
        if not self.__is_training:
            print("Recording is started")
            dpg.set_value("training_status", "Recording")
            self.__is_training = True
            self.__trainee_name = dpg.get_value("trainee_name")
            self.__trainer_name = dpg.get_value("trainer_name")
            self.start_training_time = time.time()
            print("Trainee name: " + self.__trainee_name + " ,Trainer name: " + self.__trainer_name)
        elif self.__is_training:
            print("Recording is stopped")
            dpg.set_value("training_status", "Not Recording")
            self.__is_training = False
            total_time = (time.time() - self.start_training_time) * 1  # Convert to seconds

            dpg.set_value("training_time", round(total_time,6))
            dpg.set_value("trainee_name_summary", self.__trainee_name)
            dpg.set_value("trainer_name_summary", self.__trainer_name)
            
            # Save data to .csv file
            if not os.path.exists("data"):
                os.makedirs("data")
            file_name = "data/" + self.__trainee_name + "_" + self.__trainer_name + "_" + str(round(total_time, 2)) + "_" + str(SAMPLING_RATE) + ".csv"
            with open(file_name, 'w') as f:
                f.write("Trainee Name: " + self.__trainee_name + "\n")
                f.write("Trainer Name: " + self.__trainer_name + "\n")
                f.write("Total Time Used: " + str(round(total_time, 2)) + " seconds\n")
                f.write("Sampling Rate: " + str(SAMPLING_RATE) + " sps\n")
                f.write("Pitch, Roll\n")
                for i in range(len(self.__angle_buffer[0])):
                    f.write(str(self.__angle_buffer[0][i]) + "," + str(self.__angle_buffer[1][i]) + "\n")
            print("Data is saved to " + file_name)
            
            
    def run(self):
        while True:
            self.update()
            # time.sleep(0.001)

    def start(self):
        self.__thread = threading.Thread(target=self.run)
        self.__thread.start()


def ui_init():
    dpg.create_context()
    dpg.create_viewport(title='Syringe Angle Monitoring System', height=1080, width=1920)
    dpg.setup_dearpygui()

def ui_draw():
    arduino_ui = ArduinoController()
    arduino_ui.render()
    arduino_ui.start()
    
def ui_startRenderer():
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
    
def main():
    ui_init()
    ui_draw()
    ui_startRenderer()


if __name__ == '__main__':
    main()