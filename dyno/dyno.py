import os
import argparse
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import AutoMinorLocator
import datetime
import csv
import glob
import platform
from collections import deque
from threading import Thread, Lock
import numpy.ma as ma

# Configuration variables
BAUD_RATE = 460800
LINE_WIDTH = 0.8  # Line width for individual plots
COMBINED_LINE_WIDTH = 0.4  # Line width for combined plot
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
    def __init__(self, log_file=None, print_data=False):
        self.print_data = print_data
        self.log_file = log_file
        self.num_motors = self.determine_num_motors()
        self.expected_values = 3 + 2 * self.num_motors  # time, voltage, current, and (throttle, rpm) for each motor

        px = 1/plt.rcParams['figure.dpi']  # pixel in inches
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, figsize=(1200*px, 800*px), sharex=True)
        self.lines = []
        self.data = deque(maxlen=MAX_DATA_POINTS)
        self.new_run = True
        self.mask_starts = []  # Initialize mask_starts here
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
        self.line_width = LINE_WIDTH  # Add this line
        self.mask_starts = []  # Initialize mask_starts here  # Initialize mask_starts here
        self.buffer = []  # Add this line to store data between updates
        self.update_flag = False  # Add this line to control when to update the plot

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
            self.output_file = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".log"
            self.png_file = self.output_file.rsplit('.', 1)[0] + '.png'
            self.running = True
            self.data_thread = Thread(target=self.read_serial_data)
            self.data_thread.start()
        self.mask_starts = []

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
        prev_timestamp = None
        with open(self.log_file, 'r') as f:
            for row in csv.reader(f):
                if row and not any(c.isalpha() for c in ''.join(row)):
                    if len(row) == self.expected_values and all(self.is_number(x) for x in row):
                        timestamp = int(row[0])
                        if prev_timestamp is not None and timestamp < prev_timestamp:
                            self.mask_starts.append(len(self.data))  # Add this line to detect new runs
                        self.process_data(row)
                        max_timestamp = max(max_timestamp, timestamp)
                        prev_timestamp = timestamp
        print(f"Finished reading log file. Total data points: {len(self.data)}")
        print(f"Max timestamp value: {max_timestamp}")
        if max_timestamp > 10000:
            print("Timestamp is too large")
            os._exit(0)  # Exit the script immediately
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
            return

        # Parse basic data
        time_ms, voltage_mv, pusher_current_ma = int(row[0]), float(row[1]), float(row[2])
        voltage = voltage_mv / 1000

        # Parse motor data
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
            self.max_rpm_overall = max(self.max_rpm_overall, rpm)

        # Initialize timing if needed
        if self.start_time is None:
            self.start_time = time_ms
            self.start_voltage = voltage
        relative_time = time_ms - self.start_time

        # Store data and update voltage stats
        with self.data_lock:
            self.data.append((relative_time, voltage, pusher_current_ma, throttles, rpms))
        
        self.min_voltage = min(self.min_voltage, voltage)
        self.end_voltage = voltage

        # Check for new run, reset timing data
        if len(self.data) > 1 and self.data[-1][0] < self.data[-2][0]:
            print("New run detected")
            self.mask_starts.append(len(self.data))
            self.start_time = time_ms

    def read_serial_data(self):
        while self.running:
            try:
                line = self.ser.readline()
                try:
                    decoded_line = line.decode().strip()
                    if decoded_line:
                        if self.print_data:
                            print(decoded_line)
                        with open(self.output_file, 'a') as f:
                            f.write(decoded_line + '\n')

                        if decoded_line == "end of telemetry":
                            for row in self.buffer:
                                self.process_data(row)
                            self.buffer.clear()
                            self.update_flag = True
                        else:
                            row = decoded_line.split(',')
                            if self.num_motors == 0:
                                self.num_motors = (len(row) - 3) // 2
                                self.expected_values = 3 + 2 * self.num_motors
                                self.min_throttles = [float('inf')] * self.num_motors
                                self.max_throttles = [float('-inf')] * self.num_motors
                                self.max_rpms = [float('-inf')] * self.num_motors

                            if len(row) == self.expected_values and all(self.is_number(x) for x in row):
                                self.buffer.append(row)
                except UnicodeDecodeError:
                    print("Error decoding line from serial port")
                    pass
            except Exception as e:
                print(f"Error reading serial data: {e}")

    def update_plot(self, frame):
        # Only update if we're reading from a log file or have received new telemetry
        if not (self.log_file or self.update_flag):
            return
        
        with self.data_lock:
            if len(self.data) == 0:
                return

            x = [d[0] for d in self.data]  # Time in ms
            voltage = [d[1] for d in self.data]
            pusher_current = [d[2] for d in self.data]
            throttles = list(zip(*[d[3] for d in self.data]))
            rpms = list(zip(*[d[4] for d in self.data]))

            # Apply masks if mask_starts is not empty
            if self.mask_starts:
                voltage = ma.array(voltage)
                pusher_current = ma.array(pusher_current)
                throttles = [ma.array(t) for t in throttles]
                rpms = [ma.array(r) for r in rpms]
                for start in self.mask_starts:
                    voltage[start] = ma.masked
                    pusher_current[start] = ma.masked
                    for t in throttles:
                        t[start] = ma.masked
                    for r in rpms:
                        r[start] = ma.masked

            if not self.lines:
                for i in range(len(throttles)):
                    self.lines.append(self.ax1.plot(x, throttles[i], label=f'Throttle {i+1}', linewidth=self.line_width)[0])
                    self.lines.append(self.ax2.plot(x, rpms[i], label=f'RPM {i+1}', linewidth=self.line_width)[0])
                self.lines.append(self.ax3.plot(x, voltage, label='Battery Voltage', linewidth=self.line_width)[0])
                self.lines.append(self.ax4.plot(x, pusher_current, label='Pusher Current', linewidth=self.line_width)[0])
            else:
                for i in range(len(throttles)):
                    self.lines[i*2].set_data(x, throttles[i])
                    self.lines[i*2 + 1].set_data(x, rpms[i])
                self.lines[-2].set_data(x, voltage)
                self.lines[-1].set_data(x, pusher_current)

            for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
                ax.relim()
                ax.autoscale_view()
                ax.set_ylim(bottom=0)
                ax.grid(True, which='both', linestyle='--', alpha=0.7)
                ax.yaxis.set_minor_locator(AutoMinorLocator(2))  # Add minor ticks

            self.ax1.set_ylabel('Throttle Value')
            self.ax1.set_ylim(0, 2050)
            
            # Set RPM y-axis limit with some headroom
            rpm_limit = max(1000, int(self.max_rpm_overall * 1.1))  # At least 1000, or 10% above max observed RPM
            self.ax2.set_ylim(0, rpm_limit)
            self.ax2.set_ylabel('RPM')


            self.ax3.set_ylabel('Battery Voltage (V)')
            self.ax3.set_ylim(bottom=0)  # Set the bottom limit to 0
            
            self.ax4.set_ylabel('Pusher Current (mA)')
            self.ax4.set_ylim(bottom=0)  # Set the bottom limit to 0

            self.ax4.set_xlabel('Time (ms)')
            
            # Update legend positions
            self.ax1.legend(loc='lower right')
            self.ax2.legend(loc='lower right')

            # Set x-axis to exactly fit the data
            if x:
                self.ax1.set_xlim(0, max(x))

            if self.min_max_text:
                self.min_max_text.remove()
            
            # Check if voltage values are available before formatting
            voltage_text = ""
            if self.start_voltage is not None and self.min_voltage is not None and self.end_voltage is not None:
                voltage_text = f"Voltage: Start {self.start_voltage:.2f}V, Min {self.min_voltage:.2f}V, End {self.end_voltage:.2f}V\n"
            
            self.min_max_text = self.fig.text(0.02, 0.98, 
                voltage_text +
                f"Min Throttle: " + ", ".join(f"{t:.0f}" for t in self.min_throttles) + "\n"
                f"Max Throttle: " + ", ".join(f"{t:.0f}" for t in self.max_throttles) + "\n"
                f"Max RPM: " + ", ".join(f"{rpm:.0f}" for rpm in self.max_rpms),
                verticalalignment='top', horizontalalignment='left',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            max_current = max(pusher_current)
            self.ax4.set_ylim(0, max_current * 1.1)  # Add 10% headroom

        self.save_plot_as_png()
        self.update_flag = False
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
            # Change interval from 100ms to 1000ms (1 second)
            ani = FuncAnimation(self.fig, self.update_plot, interval=1000, cache_frame_data=False)
            plt.show()  # This will block until the window is closed
        
        self.running = False
        if hasattr(self, 'data_thread'):
            self.data_thread.join()

def process_all_logs():
    log_files = glob.glob('*.log')

    for log_file in log_files:
        print(f"Processing {log_file}")
        plotter = DynamometerPlotter(log_file)
        plotter.update_plot(0)
        plt.close(plotter.fig)

    print("Finished processing all log files")



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file", nargs="?", default=None)
    parser.add_argument("-p", "--print", action="store_true", help="Print serial data to terminal")
    parser.add_argument("-a", "--all", action="store_true", help="Process all log files")
    args = parser.parse_args()

    if args.all:
        process_all_logs()
        os._exit(0)  #  sys.exit(0) doesn't work here for some reason
    else:
        plotter = DynamometerPlotter(args.log_file, print_data=args.print)
        plotter.run()

    print("Script reached end")
