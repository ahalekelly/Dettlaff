# Dettlaff Dynamometer

Captures telemetry data over the serial bus, saves it to a .log file, and plots it to a .png file

![2024-08-26_235631](https://github.com/user-attachments/assets/3997ee16-9f05-4feb-826f-fdf48b780678)

## Setup:

Make sure you have python installed

    pip install virtualenv

`cd` to the dyno directory

Create a new venv called venv and install the requirements:

Linux/Mac:
    python3 -m venv venv && venv/bin/python -m pip install -r requirements.txt

Windows Powershell:
    python -m venv venv && venv\Scripts\python -m pip install -r requirements.txt

## Usage Examples:

`cd` to the dyno directory

**To capture and plot the serial telemetry:**

Linux/Mac: `venv/bin/python dyno.py`

Windows Powershell: `venv\Scripts\python dyno.py`


**To regenerate the png files for all log files - the formatting on the real-time plots is a little off right now, this will fix them**

Linux/Mac: `venv/bin/python dyno.py -a`

Windows Powershell: `venv\Scripts\python dyno.py -a`
