#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

time = [None] * 7
encoder = [None] * 7
sea = [None] * 7
velocity = [None] * 7
motor_dir = [1, 1, 1, 1, 1, -1, -1]


def main():
    df = pd.read_csv('output7.csv')
    for i in range(7):
        time[i], encoder[i], sea[i], velocity[i] = get_motor(df, i)

    print("max sea:", max(sea[3]))
    # plt.subplot(2, 1, 1)
    plt.title("Motor displacement")
    plt.xlabel('time (s)')
    plt.ylabel('angular velocity (rad/s)')
    plt.plot(time[3], velocity[3], label='motor value 3', color="b")
    plt.plot(time[4], velocity[4], label='motor value 4', color="r")
    plt.plot(time[5], velocity[5], label='motor value 5', color="g")
    plt.plot(time[6], velocity[6], label='motor value 6', color="y")
    plt.legend(loc="upper left")

    # plt.subplot(2, 1, 2)
    # plt.title("SEA displacement")
    # plt.xlabel('time (s)')
    # plt.ylabel('angular displacement (rad)')
    

    plt.show()


def get_motor(df, num):
    data = df.to_numpy()
    vel = np.zeros(data[:, 0].shape[0])

    for i in range(1, data[:, 0].shape[0]):
        vel[i] = (data[i, num+1] - data[i-1, num+1])/(data[i, 0] - data[i-1, 0])

    box_pts = 3
    box = np.ones(box_pts)/box_pts
    vel = np.convolve(vel, box, mode='same')
    return data[:, 0], data[:, num+1], data[:, num+8]/100, vel * motor_dir[num]


if __name__ == '__main__':
    sys.exit(main())
    
    
