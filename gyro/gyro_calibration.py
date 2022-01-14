######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU6050 board
# (accelerometer/gyroscope)
# and solves for calibration coefficients for the
# gyroscope
#
#
######################################################
import numpy as np
import matplotlib.pyplot as plt

from mpu6050 import MPU6050


def gyro_cal():
    print("-" * 50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [mpu.get_gyro_data() for ii in range(0, cal_size)]  # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0, 0.0, 0.0]
    while True:
        try:
            wx, wy, wz = mpu.get_gyro_data()  # get gyro values
        except:
            continue

        mpu_array.append([wx, wy, wz])  # gyro vector append
        if np.shape(mpu_array)[0] == cal_size:
            for qq in range(0, 3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:, qq])  # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets


if __name__ == '__main__':
    mpu = MPU6050(0x68)  # IMU (Accelerometer, Gyroscope)

    cal_size = 500  # points to use for calibration

    # Gyroscope Offset Calculation
    gyro_labels = ['w_x', 'w_y', 'w_z']  # gyro labels for plots
    gyro_offsets = gyro_cal()  # calculate gyro offsets

    # record new data
    data = np.array([mpu.get_gyro_data() for ii in range(0, cal_size)])

    # plot with and without offsets
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2, 1, figsize=(12, 9))
    for ii in range(0, 3):
        axs[0].plot(data[:, ii],
                    label='${}$, Uncalibrated'.format(gyro_labels[ii]))
        axs[1].plot(data[:, ii] - gyro_offsets[ii],
                    label='${}$, Calibrated'.format(gyro_labels[ii]))
    axs[0].legend(fontsize=14)
    axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$w_{x, y, z}$ [$^{\circ}/s$]', fontsize=18)  # noqa: W605
    axs[1].set_ylabel('$w_{x, y, z}$ [$^{\circ}/s$]', fontsize=18)  # noqa: W605
    axs[1].set_xlabel('Sample', fontsize=18)
    axs[0].set_ylim([-2, 2])
    axs[1].set_ylim([-2, 2])
    axs[0].set_title('Gyroscope Calibration Offset Correction', fontsize=22)
    fig.savefig('gyro_calibration_output.png', dpi=300,
                bbox_inches='tight', facecolor='#FCFCFC')
    fig.show()
