######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU6050 board
# (accelerometer/gyroscope)
# and solves for calibration coefficients for the
# gyroscope and uses them to integrate over a test
# rotation, which approximates angular displacement
#
#
######################################################
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

from mpu6050 import MPU6050


def gyro_cal():
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [mpu.get_gyro_data() for ii in range(0, cal_size)]  # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0, 0.0, 0.0]
    while True:
        try:
            wx, wy, wz = mpu.get_gyro_data()
        except:
            continue

        mpu_array.append([wx, wy, wz])

        if np.shape(mpu_array)[0] == cal_size:
            for qq in range(0, 3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:, qq])  # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets


if __name__ == '__main__':
    mpu = MPU6050(0x68)  # IMU (Accelerometer, Gyroscope)

    # Gyroscope Offset Calculation
    gyro_labels = ['\omega_x', '\omega_y', '\omega_z']  # gyro labels for plots # noqa: W605
    cal_size = 500  # points to use for calibration
    gyro_offsets = gyro_cal()  # calculate gyro offsets

    # record new data
    input("Press Enter and Rotate Gyro 360 degrees")
    print("Recording Data...")
    record_time = 5  # how long to record
    data, t_vec = [], []
    t0 = time.time()
    while time.time()-t0 < record_time:
        data.append(mpu.get_gyro_data())
        t_vec.append(time.time()-t0)
    samp_rate = np.shape(data)[0]/(t_vec[-1]-t_vec[0])  # sample rate
    print("Stopped Recording\nSample Rate: {0:2.0f} Hz".format(samp_rate))

    # offset and integration of gyro and plotting results
    rot_axis = 2  # axis being rotated (2 = z-axis)
    data_offseted = np.array(data)[:, rot_axis]-gyro_offsets[rot_axis]
    integ1_array = cumtrapz(data_offseted, x=t_vec)  # integrate once in time

    # print out results
    print("Integration of {} in {}".format(gyro_labels[rot_axis],
                                           gyro_labels[rot_axis].split("_")[1]) +
          "-dir: {0:2.2f}m".format(integ1_array[-1]))

    # plotting routine
    plt.style.use('ggplot')
    fig, axs = plt.subplots(2, 1, figsize=(12, 9))
    axs[0].plot(t_vec, data_offseted, label="$"+gyro_labels[rot_axis]+"$")
    axs[1].plot(t_vec[1:], integ1_array,
                label=r"$\theta_"+gyro_labels[rot_axis].split("_")[1]+"$")
    [axs[ii].legend(fontsize=16) for ii in range(0, len(axs))]
    axs[0].set_ylabel('Angular Velocity, $\omega_{}$ [$^\circ/s$]'.  # noqa: W605
                      format(gyro_labels[rot_axis].split("_")[1]), fontsize=16)
    axs[1].set_ylabel(r'Rotation, $\theta_{}$ [$^\circ$]'.
                      format(gyro_labels[rot_axis].split("_")[1]), fontsize=16)
    axs[1].set_xlabel('Time [s]', fontsize=16)
    axs[0].set_title('Gyroscope Integration over 180$^\circ$ Rotation',  # noqa: W605
                     fontsize=18)
    fig.savefig('gyroscope_integration_180deg_rot.png', dpi=300,
                bbox_inches='tight', facecolor='#FFFFFF')
    plt.show()
