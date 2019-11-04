# -*- coding: utf-8 -*-
"""
Author: Atoosa Karimi
Date: September 17, 2019
Description: Reads the data output to the serial monitor, adds time stamps and 
outputs the data to a .csv file.
"""

# modules
import serial
import datetime
import os.path

# constants 
serial_port = 'COM8'; # change according to the COM being used
baud_rate = 9600;
file_path = "C:/VacuumReadData/" + str(datetime.date.today()) + ".csv"
header_string = "A1 Voltage [V], Convectron Pressure [Torr]"
# object initializations
file_preexist_flag = os.path.exists(file_path)
output_file = open(file_path, "a+");
ser = serial.Serial(serial_port, baud_rate)

try:
    while True:
        output_file = open(file_path, "a+");
        line = ser.readline();
        line = line.decode("utf-8").encode("windows-1252").decode("utf-8") #ser.readline returns a binary, convert to string
        line = line.rstrip()
        boolean = line == header_string
        if line == header_string and not file_preexist_flag:
            print("Time [s],"+ line + "\n")
            output_file.write("Time [s],"+ line + "\n")
        elif line == header_string and file_preexist_flag:
            continue
        else:
            now = datetime.datetime.now()
            seconds_since_midnight = (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
            print(str(int(seconds_since_midnight)) + "," + line + "\n")
            output_file.write(str(int(seconds_since_midnight)) + "," + line + "\n")
            output_file.close()
except KeyboardInterrupt:
    output_file.close()
    pass