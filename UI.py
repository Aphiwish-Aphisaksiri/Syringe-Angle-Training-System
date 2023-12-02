import dearpygui.dearpygui as dpg
import threading
import math
import serial
import numpy as np

class ArduinoController:

    def __init__(self):
        self.ser = serial.Serial('COM13', 115200)
        self.__gyro_buffer = [[],[],[]]
        self.__accelerometer_buffer = [[],[],[]]
        self.data_x = np.arange(0,len(self.__gyro_buffer),1)
        self.gyroX = np.array(np.zeros(len(self.__gyro_buffer)))
        self.gyroY = np.array(np.zeros(len(self.__gyro_buffer)))
        self.gyroZ = np.array(np.zeros(len(self.__gyro_buffer)))
        self.resolution = 2 #Bytes (At least 2)
        
    def create_package_object(self, header_type, payload):
        # convert package_object to bytearray
        package_bytes = bytearray(header_type.value + payload)
        print(package_bytes)
        return package_bytes
        
    def render(self):
        y_min = -32768
        y_max = 32767               
        # Plot gyro value
        with dpg.window(label="Plotter", height=1080, width=800):
            with dpg.plot(label="PlotX", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_gyroX = dpg.add_plot_axis(dpg.mvXAxis, label="x", tag="x_axis_gyroX", no_tick_labels=False)
                y_axis_gyroX = dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="y_axis_gyroX")
                dpg.set_axis_limits(y_axis_gyroX, y_min, y_max)
                dpg.add_line_series(self.data_x, self.gyroX, label="Gyro X axis", parent="y_axis_gyroX", tag="gyro_plotX")
            
            with dpg.plot(label="PlotY", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_gyroY = dpg.add_plot_axis(dpg.mvXAxis, label="x", tag="x_axis_gyroY", no_tick_labels=False)
                y_axis_gyroY = dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="y_axis_gyroY")
                dpg.set_axis_limits(y_axis_gyroY, y_min, y_max)
                dpg.add_line_series(self.data_x, self.gyroY, label="Gyro Y axis", parent="y_axis_gyroY", tag="gyro_plotY")
            
            with dpg.plot(label="PlotZ", height=333, width=-1):
                dpg.add_plot_legend()
                x_axis_gyroZ = dpg.add_plot_axis(dpg.mvXAxis, label="x", tag="x_axis_gyroZ", no_tick_labels=False)
                y_axis_gyroZ = dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="y_axis_gyroZ")
                dpg.set_axis_limits(y_axis_gyroZ, y_min, y_max)
                dpg.add_line_series(self.data_x, self.gyroZ, label="Gyro Z axis", parent="y_axis_gyroZ", tag="gyro_plotZ")
                
        with dpg.window(label="Value Monitor", height=400, width=300):
            dpg.add_text("Gyro X axis")
            dpg.add_text(tag="gyroX_value")
            dpg.add_text("Gyro Y axis")
            dpg.add_text(tag="gyroY_value")
            dpg.add_text("Gyro Z axis")
            dpg.add_text(tag="gyroZ_value")

    def update(self): # update input from potentiometer and plotting and handle sync mode
        try:
            inputdata = self.ser.read(6 * self.resolution)
            for i in range(len(self.__gyro_buffer)):
                self.__gyro_buffer[i].append(int.from_bytes(inputdata[i * self.resolution : (i + 1) * self.resolution], byteorder='little', signed=True))
                if len(self.__gyro_buffer[i]) >= 3000:
                    self.__gyro_buffer[i].pop(0)
            for i in range(len(self.__accelerometer_buffer)):
                self.__accelerometer_buffer[i].append(int.from_bytes(inputdata[(i + 3) * self.resolution : (i + 4) * self.resolution], byteorder='little', signed=True))
                if len(self.__accelerometer_buffer[i]) >= 3000:
                    self.__accelerometer_buffer[i].pop(0)

            # Plot gyro value
            self.data_x = np.arange(0, len(self.__gyro_buffer[0]), 1)
            dpg.set_value("gyro_plotX", [self.data_x, self.__gyro_buffer[0]])
            dpg.set_value("gyro_plotY", [self.data_x, self.__gyro_buffer[1]])
            dpg.set_value("gyro_plotZ", [self.data_x, self.__gyro_buffer[2]])
            dpg.fit_axis_data("x_axis_gyroX")
            dpg.fit_axis_data("y_axis_gyroX")
            dpg.fit_axis_data("x_axis_gyroY")
            dpg.fit_axis_data("y_axis_gyroY")
            dpg.fit_axis_data("x_axis_gyroZ")
            dpg.fit_axis_data("y_axis_gyroZ")
            
            dpg.set_value("gyroX_value", self.__gyro_buffer[0][-1])
            dpg.set_value("gyroY_value", self.__gyro_buffer[1][-1])
            dpg.set_value("gyroZ_value", self.__gyro_buffer[2][-1])
        
        except Exception as e:
            print(e)
            
    def run(self):
        while True:
            self.update()
            # time.sleep(0.001)

    def start(self):
        self.__thread = threading.Thread(target=self.run)
        self.__thread.start()


def ui_init():
    dpg.create_context()
    dpg.create_viewport(title='Main UI', height=1080, width=1200)
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