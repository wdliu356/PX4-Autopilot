#!/bin/bash
# This script is used for data analysis
echo "This is a data analysis script"
echo "The data to be analysed is in the $1"

# Download the necessary packages
echo "Download necesary packages"
pip install pyulog
pip install matplotlib
echo "Download complete"

# Extract necessary csv file from ulog file
echo "Extracting csv file from ulog file"
ulog2csv -m trisonica_status $1
ulog2csv -m vehicle_global_position $1

# Run the data analysis script
echo "Running data analysis script"
python3 data_analysis.py $1





