import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import datetime
import time
import csv
import sys
import os
import glob
import platform
from collections import deque
from threading import Thread, Lock

# Configuration variables
BAUD_RATE = 460800
MAX_DATA_POINTS = 10000

def get_highest_port():
    system = platform.system()
    if system == "Windows":
        ports = [p.device for p in serial.tools.list_ports.comports() if p.device.startswith("COM")]
    elif system == "Linux":
        ports = [p.device for p in serial.tools.list_ports.comports() if p.device.startswith("/dev/ttyUSB")]
    else:  # Assume macOS or other Unix-like system
        ports = [p.device for p in serial.tools.list_ports.comports() if p.device.startswith("/dev/cu.usbserial-")]
    
    if not ports:
        return None
    return max(ports, key=lambda x: int(''.join(filter(str.isdigit, x))) if any(c.isdigit() for c in x) else 0)

class DynamometerPlotter:
    def __init__(self, log_file=None, batch_mode=False):
        self.log_file = log_file
        self.batch_mode = batch_mode
        self.num_motors = self.determine_num_motors()
        self.expected_values = 3 + 2 * self.num_motors  # time, voltage, current, and (throttle, rpm) for each motor

        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, figsize=(12, 20), sharex=True)
        self.lines = []
        self.data = deque(maxlen=MAX_DATA_POINTS)
        self.last_update = time.time()
        self.new_run = True
        self.data_lock = Lock()
        self.min_max_text = None
        self.start_time = None
        self.start_voltage = None
        self.min_voltage = float('inf')
        self.end_voltage = None
        self.min_throttles = [float('inf')] * self.num_motors
        self.max_throttles = [float('-inf')] * self.num_motors
        self.max_rpms = [float('-inf')] * self.num_motors
        self.max_rpm_overall = 0  # Track the overall maximum RPM

        if log_file:
            print(f"Reading from log file: {log_file}")
            self.read_from_log()
            self.png_file = log_file.rsplit('.', 1)[0] + '.png'
        else:
            self.serial_port = get_highest_port()
            if not self.serial_port:
                raise ValueError("No suitable serial port found.")
            print(f"Using serial port: {self.serial_port}")
            self.ser = serial.Serial(self.serial_port, BAUD_RATE, timeout=0.1)
            self.output_file = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + ".log"
            self.png_file = self.output_file.rsplit('.', 1)[0] + '.png'
            self.running = True
            self.data_thread = Thread(target=self.read_serial_data)
            self.data_thread.start()

    def determine_num_motors(self):
        if self.log_file:
            with open(self.log_file, 'r') as f:
                for line in f:
                    if line.strip() and not any(c.isalpha() for c in line):
                        values = line.strip().split(',')
                        return (len(values) - 3) // 2  # Subtract time, voltage, and pusherCurrent, divide by 2 for (throttle, rpm) pairs
        else:
            # For real-time data, we'll determine this when we receive the first data point
            return 0

    def read_from_log(self):
        max_timestamp = 0  # Initialize max_timestamp
        with open(self.log_file, 'r') as f:
            for row in csv.reader(f):
                if row and not any(c.isalpha() for c in ''.join(row)):
                    if len(row) == self.expected_values and all(self.is_number(x) for x in row):
                        self.process_data(row)
                        max_timestamp = max(max_timestamp, int(row[0]))  # Update max_timestamp as int
        print(f"Finished reading log file. Total data points: {len(self.data)}")
        print(f"Max timestamp value: {max_timestamp}")  # Print max_timestamp
        self.update_plot(0)  # Ensure the plot is updated after reading the log file

    def is_number(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def process_data(self, row):
        if len(row) != self.expected_values:
            print(f"Ignoring row with unexpected number of values: {row}")
            return  # Ignore lines with unexpected number of values

        time_ms, voltage_mv, pusher_current_ma = int(row[0]), float(row[1]), float(row[2])
        voltage = voltage_mv / 1000

        throttles = []
        rpms = []
        
        for i in range(self.num_motors):
            throttle = float(row[3 + i*2])
            rpm = float(row[4 + i*2])
            throttles.append(throttle)
            rpms.append(rpm)
            self.min_throttles[i] = min(self.min_throttles[i], throttle)
            self.max_throttles[i] = max(self.max_throttles[i], throttle)
            self.max_rpms[i] = max(self.max_rpms[i], rpm)
            self.max_rpm_overall = max(self.max_rpm_overall, rpm)  # Update overall max RPM

        if self.start_time is None:
            self.start_time = time_ms
            self.start_voltage = voltage
        relative_time = time_ms - self.start_time

        with self.data_lock:
            self.data.append((relative_time, voltage, pusher_current_ma, throttles, rpms))
        
        self.min_voltage = min(self.min_voltage, voltage)
        self.end_voltage = voltage

        if len(self.data) > 1 and self.data[-1][0] < self.data[-2][0]:
            print("New run detected")
            self.new_run = True
            self.start_time = time_ms
            self.start_voltage = voltage
            self.min_voltage = voltage
            self.end_voltage = voltage
            self.min_throttles = [float('inf')] * self.num_motors
            self.max_throttles = [float('-inf')] * self.num_motors
            self.max_rpms = [float('-inf')] * self.num_motors
            self.max_rpm_overall = 0

    def read_serial_data(self):
        while self.running:
            try:
                line = self.ser.readline()
                try:
                    decoded_line = line.decode().strip()
                    if decoded_line:
                        with open(self.output_file, 'a') as f:
                            f.write(decoded_line + '\n')

                        row = decoded_line.split(',')
                        if self.num_motors == 0:
                            self.num_motors = (len(row) - 3) // 2
                            self.expected_values = 3 + 2 * self.num_motors
                            self.min_throttles = [float('inf')] * self.num_motors
                            self.max_throttles = [float('-inf')] * self.num_motors
                            self.max_rpms = [float('-inf')] * self.num_motors

                        if len(row) == self.expected_values and all(self.is_number(x) for x in row):
                            self.process_data(row)
                except UnicodeDecodeError:
                    print("Error decoding line from serial port")
                    pass
            except Exception as e:
                print(f"Error reading serial data: {e}")

    def update_plot(self, frame):
        with self.data_lock:
            if len(self.data) == 0:
                return

            if self.new_run:
                self.lines = []
                self.new_run = False

            x = [d[0] for d in self.data]  # Time in ms
            voltage = [d[1] for d in self.data]
            pusher_current = [d[2] for d in self.data]
            throttles = list(zip(*[d[3] for d in self.data]))
            rpms = list(zip(*[d[4] for d in self.data]))

            if not self.lines:
                print("Creating new plot lines")
                for i in range(self.num_motors):
                    self.lines.append(self.ax1.plot(x, throttles[i], label=f'Throttle {i+1}')[0])
                    self.lines.append(self.ax2.plot(x, rpms[i], label=f'RPM {i+1}')[0])
                self.lines.append(self.ax3.plot(x, voltage, label='Battery Voltage')[0])
                self.lines.append(self.ax4.plot(x, pusher_current, label='Pusher Current')[0])

                # Save the plot as PNG
                self.save_plot_as_png()

            else:
                for i in range(self.num_motors):
                    self.lines[i*2].set_data(x, throttles[i])
                    self.lines[i*2 + 1].set_data(x, rpms[i])
                self.lines[-2].set_data(x, voltage)
                self.lines[-1].set_data(x, pusher_current)

            for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
                ax.relim()
                ax.autoscale_view()
                ax.set_ylim(bottom=0)
                ax.grid(True, which='both', linestyle='--', alpha=0.7)

            self.ax1.set_ylabel('Throttle Value')
            self.ax1.set_ylim(0, 2050)  # Increased to 2050
            
            # Set RPM y-axis limit with some headroom
            rpm_limit = max(1000, int(self.max_rpm_overall * 1.1))  # At least 1000, or 10% above max observed RPM
            self.ax2.set_ylim(0, rpm_limit)
            self.ax2.set_ylabel('RPM')
            self.ax3.set_ylabel('Battery Voltage (V)')
            self.ax3.set_ylim(bottom=0)  # Set the bottom limit to 0
            
            self.ax4.set_ylabel('Pusher Current (mA)')
            self.ax4.set_ylim(bottom=0)  # Set the bottom limit to 0
            self.ax3.set_ylabel('Battery Voltage (V)')
            self.ax3.set_ylim(bottom=0)  # Set the bottom limit to 0
            
            self.ax4.set_ylabel('Pusher Current (mA)')
            self.ax4.set_ylim(bottom=0)  # Set the bottom limit to 0

            self.ax4.set_xlabel('Time (ms)')
            
            # Update legend positions
            self.ax3.legend(loc='lower right')
            self.ax4.legend(loc='lower right')

            # Set x-axis to exactly fit the data
            if x:
                self.ax1.set_xlim(0, max(x))

            if self.min_max_text:
                self.min_max_text.remove()
            self.min_max_text = self.fig.text(0.02, 0.98, 
                f"Voltage: Start {self.start_voltage:.2f}V, Min {self.min_voltage:.2f}V, End {self.end_voltage:.2f}V\n"
                f"Min Throttle: " + ", ".join(f"{t:.0f}" for t in self.min_throttles) + "\n"
                f"Max Throttle: " + ", ".join(f"{t:.0f}" for t in self.max_throttles) + "\n"
                f"Max RPM: " + ", ".join(f"{rpm:.0f}" for rpm in self.max_rpms),
                verticalalignment='top', horizontalalignment='left',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        return self.lines + [self.min_max_text]

    def save_plot_as_png(self):
        if not hasattr(self, 'png_file'):
            self.png_file = self.log_file.rsplit('.', 1)[0] + '.png' if self.log_file else 'output.png'
        print(f"Saving plot as PNG: {self.png_file}")
        plt.savefig(self.png_file, dpi=300, bbox_inches='tight')

    def run(self):
        if self.log_file:
            print("Displaying plot for log file data")
            self.update_plot(0)  # Ensure plot is updated once for log files
            plt.show()
        else:
            print("Starting real-time plotting")
            ani = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
            plt.show()
        
        self.running = False
        if hasattr(self, 'data_thread'):
            self.data_thread.join()

def process_all_logs():
    log_files = glob.glob('*.log')
    for log_file in log_files:
        print(f"Processing {log_file}")
        plotter = DynamometerPlotter(log_file, batch_mode=True)
        plotter.update_plot(0)  # Generate the plot
        plotter.save_plot_as_png()  # Save the plot as PNG
        plt.close(plotter.fig)  # Close the figure to free up memory
    print("Finished processing all log files")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == '-p':
        process_all_logs()
    else:
        log_file = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith('.log') else None
        plotter = DynamometerPlotter(log_file)
        plotter.run()