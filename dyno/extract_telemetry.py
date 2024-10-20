# This script extracts telemetry data from the most recent log and exports to csv so you can import data into spreadsheet software

import csv
import os
import glob
from datetime import datetime

log_files = glob.glob('*.log')
if not log_files:
    print("No .log files found")
    exit(1)

latest_file = max(log_files, key=lambda f: datetime.strptime(os.path.basename(f).split('.')[0], "%Y-%m-%d_%H:%M:%S"))
input_file = latest_file
output_file = os.path.join(os.path.basename(input_file).rsplit('.', 1)[0] + '_telem.csv')

with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    
    for row in reader:
        if len(row) > 5:
            writer.writerow(row)

print(f"Processed file saved as {output_file}")
