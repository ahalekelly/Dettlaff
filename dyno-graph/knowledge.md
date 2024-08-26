# Dyno-Graph Knowledge

## Purpose
- The dyno-graph script is used for plotting dynamometer data for the Dettlaff project

## Key Features
- Reads data from a log file or serial port
- Supports multiple motors (determined dynamically from the data)
- Plots voltage, current, throttle, and RPM data
- Saves plots as PNG files

## Usage
- Can be run with a log file: `venv/bin/python dyno-graph.py [logfile]`
- Can also capture real-time data from a serial port

## Dependencies
- matplotlib
- pyserial

## Environment
- Uses a Python virtual environment (venv)
- Requirements are specified in requirements.txt

## Important Notes
- The script doesn't exit until the user quits it
- Code changes cannot be applied while the script is running

## TODO
- Consider optimizing the plot saving process (currently saves twice)
- Explore adding more analysis features or statistics from the data
