######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU6050 board,
# (accelerometer/gyroscope)
# and solves for calibration coefficients for the
# accelerometer
#
#
######################################################
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from mpu6050 import MPU6050


def accel_fit(x_input, m_x, b):
    return (m_x * x_input) + b  # fit equation for accel calibration


def accel_cal():
    print("-" * 50)
    print("Accelerometer Calibration")
    mpu_offsets = [[], [], []]  # offset array to be printed
    axis_vec = ['z', 'y', 'x']  # axis labels
    cal_directions = ["upward", "downward", "perpendicular to gravity"]  # direction for IMU cal
    cal_indices = [2, 1, 0]  # axis indices
    for qq, ax_qq in enumerate(axis_vec):
        ax_offsets = [[], [], []]
        print("-" * 50)
        for direc_ii, direc in enumerate(cal_directions):
            input("-" * 8 + " Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -" +
                  ax_qq + "-axis pointed " + direc)
            [mpu.get_accel_data() for ii in range(0, cal_size)]  # clear buffer between readings
            mpu_array = []
            while len(mpu_array) < cal_size:
                try:
                    ax, ay, az = mpu.get_accel_data()
                    mpu_array.append([ax, ay, az])  # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:, cal_indices[qq]]  # offsets for direction

        # use three calibrations (+1g, -1g, 0g) for linear fit
        popts, _ = curve_fit(accel_fit, np.append(np.append(ax_offsets[0],
                                                            ax_offsets[1]), ax_offsets[2]),
                             np.append(np.append(1.0 * np.ones(np.shape(ax_offsets[0])),
                                                 -1.0 * np.ones(np.shape(ax_offsets[1]))),
                                       0.0 * np.ones(np.shape(ax_offsets[2]))),
                             maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts  # place slope and intercept in offset array
    print('Accelerometer Calibration Complete')
    return mpu_offsets


if __name__ == '__main__':
    mpu = MPU6050(0x68)  # IMU (Accelerometer, Gyroscope)

    cal_size = 1000  # number of points to use for calibration

    # Accelerometer Gravity Calibration
    accel_labels = ['a_x', 'a_y', 'a_z']  # gyro labels for plots
    accel_coeffs = accel_cal()  # grab accel coefficients

    # record new data
    data = np.array([mpu.get_accel_data() for ii in range(0, cal_size)])

    # plot with and without offsets
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2, 1, figsize=(12, 9))
    for ii in range(0, 3):
        axs[0].plot(data[:, ii],
                    label='${}$, Uncalibrated'.format(accel_labels[ii]))
        axs[1].plot(accel_fit(data[:, ii], *accel_coeffs[ii]),
                    label='${}$, Calibrated'.format(accel_labels[ii]))
    axs[0].legend(fontsize=14)
    axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x, y, z}$ [g]', fontsize=18)
    axs[1].set_ylabel('$a_{x, y, z}$ [g]', fontsize=18)
    axs[1].set_xlabel('Sample', fontsize=18)
    axs[0].set_ylim([-2, 2])
    axs[1].set_ylim([-2, 2])
    axs[0].set_title('Accelerometer Calibration Calibration Correction', fontsize=18)
    fig.savefig('accel_calibration_output.png', dpi=300,
                bbox_inches='tight', facecolor='#FCFCFC')
    fig.show()
