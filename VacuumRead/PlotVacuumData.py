# -*- coding: utf-8 -*-
"""
Author: Atoosa Karimi
Date: September 18, 2019
Description: Plots the data collected from VacuumReadData.
"""
# modules
import pandas as pd
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog

# constants
header = ["Time [s]", "A1 Voltage [V]", "Convectron Pressure [Torr]"]

# prompt user for data file
root = tk.Tk()
file_path = filedialog.askopenfilename(initialdir = "C:/VacuumReadData",title = "Select a .csv file",
                                      filetypes = (("CSV Files","*.csv"),))
root.withdraw()
print("Chosen file: " + file_path);


df = pd.read_csv(file_path, delimiter = ',',usecols = header, skipinitialspace = True)[header]

plt.figure(figsize = (10,10))
plt.plot(df['Time [s]'], df['Convectron Pressure [Torr]'], label = "Convectron Sensor", color = '#6b5075')
plt.title('Pressure versus Time')
plt.xlabel('Time [s]')
plt.ylabel('Pressure [Torr]')
plt.legend(loc = 'upper left', fontsize = 'x-small')
plt.grid(axis = 'both')


