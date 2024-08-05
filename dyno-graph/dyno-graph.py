import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import datetime
import time
import csv
import sys
import os
import platform
from collections import deque
from threading import Thread, Lock

# Configuration variables
BAUD_RATE = 460800
MOTORS_PRESENT = [True, True, True, True]  # Configuration for motors 1, 2, 3, 4
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
    def __init__(self, log_file=None):
        self.log_file = log_file
        self.motors_present = MOTORS_PRESENT
        self.num_active_motors = sum(self.motors_present)
        self.expected_values = 2 + 2 * self.num_active_motors  # time, voltage, and (throttle, rpm) for each active motor

        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
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
        self.min_throttles = [float('inf')] * 4
        self.max_throttles = [float('-inf')] * 4
        self.max_rpms = [float('-inf')] * 4

        if log_file:
            print(f"Reading from log file: {log_file}")
            self.check_motor_configuration(log_file)
            self.read_from_log()
        else:
            self.serial_port = get_highest_port()
            if not self.serial_port:
                raise ValueError("No suitable serial port found.")
            print(f"Using serial port: {self.serial_port}")
            self.ser = serial.Serial(self.serial_port, BAUD_RATE, timeout=0.1)
            self.output_file = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S") + ".log"
            self.running = True
            self.data_thread = Thread(target=self.read_serial_data)
            self.data_thread.start()

    def check_motor_configuration(self, log_file):
        with open(log_file, 'r') as f:
            first_line = f.readline().strip().split(',')
            actual_values = len(first_line)
            if actual_values != self.expected_values:
                print(f"Warning: Motor configuration mismatch detected.")
                print(f"Expected {self.expected_values} values based on current MOTORS_PRESENT configuration.")
                print(f"Found {actual_values} values in the log file.")
                
                suggested_config = [False] * 4
                for i in range(min(4, (actual_values - 2) // 2)):
                    suggested_config[i] = True
                
                print(f"Suggested MOTORS_PRESENT configuration: {suggested_config}")
                print("Please update the MOTORS_PRESENT configuration in the script and run again.")
                sys.exit(1)
        print("Motor configuration check passed.")

    def read_from_log(self):
        with open(self.log_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) == self.expected_values and all(self.is_number(x) for x in row):
                    self.process_data(row)
        print(f"Finished reading log file. Total data points: {len(self.data)}")
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

        time_ms, voltage_mv = map(float, row[:2])
        voltage = voltage_mv / 1000

        throttles = [0] * 4
        rpms = [0] * 4
        
        throttle_index = 2
        rpm_index = 2 + self.num_active_motors
        for i, motor_present in enumerate(self.motors_present):
            if motor_present:
                throttle = float(row[throttle_index])
                rpm = float(row[rpm_index])
                throttles[i] = throttle
                rpms[i] = rpm
                self.min_throttles[i] = min(self.min_throttles[i], throttle)
                self.max_throttles[i] = max(self.max_throttles[i], throttle)
                self.max_rpms[i] = max(self.max_rpms[i], rpm)
                throttle_index += 1
                rpm_index += 1

        if self.start_time is None:
            self.start_time = time_ms
            self.start_voltage = voltage
        relative_time = time_ms - self.start_time

        with self.data_lock:
            self.data.append((relative_time, voltage, throttles, rpms))
        
        self.min_voltage = min(self.min_voltage, voltage)
        self.end_voltage = voltage

        if len(self.data) > 1 and self.data[-1][0] < self.data[-2][0]:
            print("New run detected")
            self.new_run = True
            self.start_time = time_ms
            self.start_voltage = voltage
            self.min_voltage = voltage
            self.end_voltage = voltage
            self.min_throttles = [float('inf')] * 4
            self.max_throttles = [float('-inf')] * 4
            self.max_rpms = [float('-inf')] * 4

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
                print("No data to plot")
                return

            if self.new_run:
                self.lines = []
                self.new_run = False

            x = [d[0] for d in self.data]  # Time in ms
            voltage = [d[1] for d in self.data]
            throttles = list(zip(*[d[2] for d in self.data]))
            rpms = list(zip(*[d[3] for d in self.data]))

            if not self.lines:
                print("Creating new plot lines")
                self.lines.append(self.ax1.plot(x, voltage, label='Battery Voltage')[0])
                for i, (throttle, motor_present) in enumerate(zip(throttles, self.motors_present)):
                    if motor_present:
                        self.lines.append(self.ax2.plot(x, throttle, label=f'Throttle {i+1}')[0])
                for i, (rpm, motor_present) in enumerate(zip(rpms, self.motors_present)):
                    if motor_present:
                        self.lines.append(self.ax3.plot(x, rpm, label=f'RPM {i+1}')[0])
            else:
                self.lines[0].set_data(x, voltage)
                idx = 1
                for i, (throttle, motor_present) in enumerate(zip(throttles, self.motors_present)):
                    if motor_present:
                        self.lines[idx].set_data(x, throttle)
                        idx += 1
                for i, (rpm, motor_present) in enumerate(zip(rpms, self.motors_present)):
                    if motor_present:
                        self.lines[idx].set_data(x, rpm)
                        idx += 1

            for ax in (self.ax1, self.ax2, self.ax3):
                ax.relim()
                ax.autoscale_view()
                ax.set_ylim(bottom=0)
                ax.grid(True, which='both', linestyle='--', alpha=0.7)

            self.ax1.set_ylabel('Battery Voltage (V)')
            self.ax2.set_ylabel('Throttle Value')
            self.ax2.set_ylim(0, 2050)  # Increased to 2050
            self.ax3.set_ylabel('RPM')
            self.ax3.set_xlabel('Time (ms)')
            
            # Update legend positions
            self.ax2.legend(loc='lower right')
            self.ax3.legend(loc='lower right')

            # Set x-axis to exactly fit the data
            if x:
                self.ax1.set_xlim(0, max(x))

            if self.min_max_text:
                self.min_max_text.remove()
            self.min_max_text = self.fig.text(0.02, 0.98, 
                f"Voltage: Start {self.start_voltage:.2f}V, Min {self.min_voltage:.2f}V, End {self.end_voltage:.2f}V\n"
                f"Min Throttle: " + ", ".join(f"{t:.0f}" for t, present in zip(self.min_throttles, self.motors_present) if present) + "\n"
                f"Max Throttle: " + ", ".join(f"{t:.0f}" for t, present in zip(self.max_throttles, self.motors_present) if present) + "\n"
                f"Max RPM: " + ", ".join(f"{rpm:.0f}" for rpm, present in zip(self.max_rpms, self.motors_present) if present),
                verticalalignment='top', horizontalalignment='left',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        return self.lines + [self.min_max_text]

    def run(self):
        if self.log_file:
            print("Displaying plot for log file data")
            plt.show()
        else:
            print("Starting real-time plotting")
            ani = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
            plt.show()
        
        self.running = False
        if hasattr(self, 'data_thread'):
            self.data_thread.join()

if __name__ == "__main__":
    log_file = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].endswith('.log') else None
    plotter = DynamometerPlotter(log_file)
    plotter.run()