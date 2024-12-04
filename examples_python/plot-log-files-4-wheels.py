#!/bin/env python3

# Usage:
# plot-log-files.py session1-mvsim_r1_logger_pose.log

# Requirements:
# pip install pandas matplotlib

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os


def main(file_path):
    # Read the main file
    try:
        main_data = pd.read_csv(file_path, skipinitialspace=True)
        print(f"Main data loaded from {file_path}")
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return

    # Define the wheel files based on the main file name
    base_name = file_path.replace("pose.log", "")
    wheel_files = [f"{base_name}wheel{i}.log" for i in range(1, 5)]

    # Load wheel data
    wheel_data = {}
    for i, wheel_file in enumerate(wheel_files, start=1):
        try:
            wheel_data[i] = pd.read_csv(wheel_file, skipinitialspace=True)
            print(f"Wheel {i} data loaded from {wheel_file}")
        except Exception as e:
            print(f"Error reading {wheel_file}: {e}")
            return

    # Create plots for the wheel files (non-blocking)
    plot_wheel_data(wheel_data)

    # Create plots for the main file (blocking)
    plot_main_data(main_data)


def plot_main_data(data):
    """Create plots for the main file data."""
    plt.figure(figsize=(12, 8))

    # print(data.columns)
    ts = data['Timestamp'].to_numpy()

    # Plot 1: qx, qy, qz over time
    plt.subplot(3, 1, 1)
    plt.plot(ts,  data['qx'].to_numpy(), label='qx')
    plt.plot(ts,  data['qy'].to_numpy(), label='qy')
    plt.plot(ts,  data['qz'].to_numpy(), label='qz')
    plt.xlabel('Timestamp')
    plt.ylabel('Position')
    plt.title('Position (qx, qy, qz) vs Time')
    plt.grid()
    plt.legend()

    # Plot 2: Orientation angles (qpitch, qroll, qyaw) over time
    plt.subplot(3, 1, 2)
    plt.plot(ts,
             data['qpitch'].to_numpy(), label='qpitch')
    plt.plot(ts,
             data['qroll'].to_numpy(), label='qroll')
    plt.plot(ts,
             data['qyaw'].to_numpy(), label='qyaw')
    plt.xlabel('Timestamp')
    plt.ylabel('Orientation')
    plt.title('Orientation Angles vs Time')
    plt.grid()
    plt.legend()

    # Plot 3: dqz over time
    plt.subplot(3, 1, 3)
    plt.plot(ts, data['dqx'].to_numpy(), label='dqx', color='red')
    plt.plot(ts, data['dqy'].to_numpy(), label='dqy', color='green')
    plt.plot(ts, data['dqz'].to_numpy(), label='dqz', color='blue')
    plt.xlabel('Timestamp')
    plt.ylabel('Velocity')
    plt.title('Velocity components')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()


def plot_wheel_data(wheel_data):
    """Create plots for the wheel data."""
    plt.figure(figsize=(12, 12))

    # Create one plot per variable group across all wheels
    variables = ['friction_x', 'friction_y',
                 'velocity_x', 'velocity_y',
                 'torque', 'weight']
    wheel_names = ['LR', 'RR', 'LF', 'RF']

    for i, var in enumerate(variables, start=1):
        plt.subplot(3, 2, i)
        for wheel, data in wheel_data.items():
            ts = data['Timestamp'].to_numpy()
            plt.plot(ts,
                     data[var].to_numpy(), label=f'{wheel_names[wheel-1]} wheel')
        plt.xlabel('Timestamp')
        plt.ylabel(var)
        plt.title(f'{var.capitalize()} vs Time')
        plt.grid()
        plt.legend()

    plt.tight_layout()
    plt.show(block=False)


if __name__ == "__main__":
    # Check if the path to the main file is provided
    if len(sys.argv) != 2:
        print("Usage: python3 script.py </path/to/session1-mvsim_r1_logger_pose.log>")
        sys.exit(1)

    file_path = sys.argv[1]
    main(file_path)
