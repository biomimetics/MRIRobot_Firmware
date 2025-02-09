#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys


def main():
    df = pd.read_csv('output2.csv')
    time, encoder, sea = get_motor(df, 1)

    print("max sea:", max(sea))
    plt.subplot(2, 1, 1)
    plt.title("Motor displacement")
    plt.xlabel('time (s)')
    plt.ylabel('angular displacement (rad)')
    plt.plot(time, encoder, label='motor value')

    plt.subplot(2, 1, 2)
    plt.title("SEA displacement")
    plt.xlabel('time (s)')
    plt.ylabel('angular displacement (rad)')
    plt.plot(time, sea, label='SEA value', color="r")

    plt.show()


def get_motor(df, num):
    data = df.to_numpy()
    return data[:, 0], data[:, num+1], data[:, num+8]/100


if __name__ == '__main__':
    sys.exit(main())
    
    
