Python Virtual Environment Instructions:

`cd` to the dyno-graph directory

# Setup:

Create a new venv called venv:
    python3 -m venv venv

Install requirements inside that venv:
    venv/bin/python -m pip install -r requirements.txt

# Usage:

To capture and plot the serial telemetry, run 
    venv/bin/python dyno-graph.py

If you want to also print the serial data while it's coming in:
    venv/bin/python dyno-graph.py -p

To regenerate the plot for an existing log file:
    venv/bin/python dyno-graph.py 2024-08-25_21:30:04.log

To regenerate the plots for all log files:
    venv/bin/python dyno-graph.py -a
