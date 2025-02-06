import matplotlib.pyplot as plt
import pandas as pd
import glob
import time
from decimal import *
import os
import csv

# Renames columns from log file for plotting presentation
def rename(df, log):
    if "imu_42688" in log:
        df = df.rename(columns={"DSP Timestamp":"IMU42688 DSP Timestamp",
                                "Temperature":"IMU42688 Temperature",
                                "Accel1": "IMU42688 Accel1",
                                "Accel2": "IMU42688 Accel2",
                                "Accel3": "IMU42688 Accel3",
                                "Gyro1" : "IMU42688 Gyro1" ,
                                "Gyro2" : "IMU42688 Gyro2" ,
                                "Gyro3" : "IMU42688 Gyro3" })
    if "pressure":
        df = df.rename(columns={"DSP Timestamp":"Barometer DSP Timestamp",
                                "Temperature" : "Barometer Temperature",
                                "Pressure"    : "Barometer Pressure"
        })
    
    return df

# Converts Epoch time to current date
def conv(epoch):
    epoch = Decimal(epoch) / 1000000000
    epoch = time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime(epoch))
    return epoch

def plot_imu(df):
    df['Timestamp'] = df['Timestamp'].apply(conv)
    df = df.set_index('Timestamp')

    cols = list(df.columns)
    axs = df[cols].plot(figsize=(25,10),subplots=True)

def plot_pressure(df):
    try:
        df['Timestamp'] = df['Timestamp'].apply(conv)
        df = df.set_index('Timestamp')

        cols = list(df.columns)
        axs = df[cols].plot(figsize=(30,15),subplots=True)
    except:
        print("Unable to plot")

def create_plots(path, duration):

    logs_to_plot = []
    
    # Contains directories to all logs which are to be plotted
    logs_dir = [
        (f"{path}/pressure/log"),
        (f"{path}/imu_42688/log")
    ]

    # Makes sure logs to be plotted have data
    for log in logs_dir:
        if len(os.listdir(log)) != 0:
            filename = str(glob.glob(log + '/*.csv')[0])
            file = open(filename)
            reader = csv.reader(file)
            lines = len(list(reader))
            if lines > 7:
                logs_to_plot.append(log)

    if len(logs_to_plot) == 0: # leave if no data is to be plotted
        return

    df = pd.DataFrame()
    for log in range(len(logs_to_plot)):
        if df.empty:
            df = pd.read_csv(str(glob.glob(logs_to_plot[log] + '/*.csv')[0]))
            df = rename(df, logs_to_plot[log])
            df['Timestamp'] = df['Timestamp'].apply(conv)
            
        else:
            df_t = pd.read_csv(str(glob.glob(logs_to_plot[log] + '/*.csv')[0]))
            df_t = rename(df_t, logs_to_plot[log])
            df_t['Timestamp'] = df_t['Timestamp'].apply(conv)
            
            df = df.merge(df_t, on="Timestamp", how="inner")

    df = df.set_index('Timestamp')
    cols = list(df.columns)

    axs = df[cols].plot(figsize=(35,15),subplots=True)

    for i, row in enumerate(axs):

        row.set_title(cols[i])
        row.get_legend().remove()

        if "Accel" in cols[i]:
            row.set_ylabel("m/s^2")
        elif "Gyro" in cols[i]:
            row.set_ylabel("deg/s")
        elif "Temperature" in cols[i]:
            row.set_ylabel("C")
        elif "Pressure" in cols[i]:
            row.set_ylabel("N/m^2")

    plt.subplots_adjust(hspace=0.7)
    plt.savefig(f'{path}/Simple_plot.png')


if __name__ == "__main__":
    create_plots("/Desktop/pulling_logs/2021-03-05-22:46:19", 5)