import csv
import os
# from statistics import mean
# import numpy as np
import matplotlib.pyplot as plt
import sys

ulg = sys.argv[1]

header = ulg[:-4]
trisonica = header + '_trisonica_status_0.csv'
global_position = header + '_vehicle_global_position_0.csv'

wind_speed = []
wind_dir = []
wind_time = []
altitude = []
altitude_time = []

with open(trisonica, 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    for row in reader:
        wind_speed.append(float(row[2]))
        wind_dir.append(float(row[12]))
        wind_time.append(float(row[0]))

## check whether the position data is available
# if not os.path.exists(global_position):
#     print('No position data available')
#     sys.exit()
if os.path.exists(global_position):
    with open(global_position, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            altitude.append(float(row[4]))
            altitude_time.append(float(row[0]))
    wind_time = [(x - wind_time[0])/1000000 for x in wind_time]
    altitude_time = [(x - altitude_time[0])/1000000 for x in altitude_time]
    # print(wind_speed)
# print(wind_dir)
# print(wind_time)
# print(altitude)
# print(altitude_time)

# plot wind_speed vs wind_time and altitude vs altitude_time and wind_dir vs wind_time in three subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
ax1.plot(wind_time, wind_speed)
ax1.set_title('Wind Speed')
ax1.set_ylabel('Speed (m/s)')
if os.path.exists(global_position):
    ax2.plot(altitude_time, altitude)
    ax2.set_title('Altitude')
    ax2.set_ylabel('Altitude (m)')
ax3.plot(wind_time, wind_dir)
ax3.set_title('Wind Direction')
ax3.set_ylabel('Direction (deg)')
ax3.set_xlabel('Time (s)')
plt.show()

