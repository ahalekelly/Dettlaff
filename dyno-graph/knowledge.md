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


# Python Virtual Environment Instructions:

`cd` to the dyno-graph directory

## Setup:

Create a new venv called venv:
    python3 -m venv venv

Install requirements inside that venv:
    venv/bin/python -m pip install -r requirements.txt

## Usage:

To capture and plot the serial telemetry, run 
    venv/bin/python dyno-graph.py

If you want to also print the serial data while it's coming in:
    venv/bin/python dyno-graph.py -p

To regenerate the plot for an existing log file:
    venv/bin/python dyno-graph.py 2024-08-25_21:30:04.log

To regenerate the plots for all log files:
    venv/bin/python dyno-graph.py all
