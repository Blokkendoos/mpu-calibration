######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag)
# and solves for calibration coefficients for the
# accelerometer, gyroscope, and magnetometer using
# various methods with the IMU attached to a 3D-cube
# (MPU9250 + calibration block)
# --- The calibration coefficients are then saved to
# --- a local .csv file, which can be loaded upon
# --- the next run of the program
#
#
######################################################
import time
import csv
import math
import numpy as np
import matplotlib.pyplot as plt

from mpu6050 import MPU6050
from hmc5883 import HMC5883

from accel_calibration import accel_cal
from gyro_calibration import gyro_cal
from mag_calibration import mag_cal


# Mag Calibration Fitting
def outlier_removal(x_ii, y_ii):
    x_diff = np.append(0.0, np.diff(x_ii))  # looking for outliers
    stdev_amt = 5.0  # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff) > np.abs(np.mean(x_diff)) +
                          (stdev_amt * np.std(x_diff)))  # outlier in x-var
    x_inliers = np.where(np.abs(x_diff) < np.abs(np.mean(x_diff)) +
                         (stdev_amt * np.std(x_diff)))
    y_diff = np.append(0.0, np.diff(y_ii))  # looking for outliers
    y_outliers = np.where(np.abs(y_diff) > np.abs(np.mean(y_diff)) +
                          (stdev_amt * np.std(y_diff)))  # outlier in y-var
    y_inliers = np.abs(y_diff) < np.abs(np.mean(y_diff)) + (stdev_amt * np.std(y_diff))  # outlier vector
    if len(x_outliers) != 0:
        x_ii[x_outliers] = np.nan  # null outlier
        y_ii[x_outliers] = np.nan  # null outlier
    if len(y_outliers) != 0:
        y_ii[y_outliers] = np.nan  # null outlier
        x_ii[y_outliers] = np.nan  # null outlier
    return x_ii, y_ii


# Plot Real-Time Values to Test
def mpu_plot_test():
    # Figure/Axis Formatting
    plt.style.use('ggplot')  # stylistic visualization
    fig = plt.figure(figsize=(12, 9))  # start figure
    axs = [[], [], [], []]  # axis vector
    axs[0] = fig.add_subplot(321)  # accel axis
    axs[1] = fig.add_subplot(323)  # gyro axis
    axs[2] = fig.add_subplot(325)  # mag axis
    axs[3] = fig.add_subplot(122, projection='polar')  # heading axis
    plt_pts = 1000  # points to plot
    y_labels = ['Acceleration [g]', 'Angular Velocity [$^\circ/s$]', 'Magnetic Field [uT]']  # noqa: W605
    for ax_ii in range(0, len(y_labels)):
        axs[ax_ii].set_xlim([0, plt_pts])  # set x-limits for time-series plots
        axs[ax_ii].set_ylabel(y_labels[ax_ii])  # set y-labels
    ax_ylims = [[-4.0, 4.0], [-300.0, 300.0], [-100.0, 100.0]]  # ax limits
    for qp in range(0, len(ax_ylims)):
        axs[qp].set_ylim(ax_ylims[qp])  # set axis limits
    axs[3].set_rlim([0.0, 100.0])  # set limits on heading plot
    axs[3].set_rlabel_position(112.5)  # offset radius labels
    axs[3].set_theta_zero_location("N")  # set north to top of plot
    axs[3].set_theta_direction(-1)  # set rotation N->E->S->W
    axs[3].set_title('Magnetometer Heading')  # polar plot title
    axs[0].set_title('Calibrated MPU Time Series Plot')  # imu time series title
    fig.canvas.draw()  # draw axes

    # Pre-allocate plot vectors
    dummy_y_vals = np.zeros((plt_pts, ))  # for populating the plots at start
    dummy_y_vals[dummy_y_vals == 0] = np.nan  # keep plots clear
    lines = []  # lines for looping updates
    for ii in range(0, 9):
        if ii in range(0, 3):  # accel pre-allocation
            line_ii, = axs[0].plot(np.arange(0, plt_pts), dummy_y_vals,
                                   label='$' + mpu_labels[ii] + '$', color=plt.cm.tab10(ii))
        elif ii in range(3, 6):  # gyro pre-allocation
            line_ii, = axs[1].plot(np.arange(0, plt_pts), dummy_y_vals,
                                   label='$' + mpu_labels[ii] + '$', color=plt.cm.tab10(ii))
        elif ii in range(6, 9):  # mag pre-allocation
            jj = ii-6
            line_jj, = axs[2].plot(np.arange(0, plt_pts), dummy_y_vals,
                                   label='$' + mpu_labels[ii] + '$', color=plt.cm.tab10(ii))
            line_ii, = axs[3].plot(dummy_y_vals, dummy_y_vals,
                                   label='$' + mag_cal_axes[jj] + '$-Axis Heading',
                                   color=plt.cm.tab20b(int(jj*4)),
                                   linestyle='', marker='o', markersize=3)
            lines.append(line_jj)
        lines.append(line_ii)
    ax_legs = [axs[tt].legend() for tt in range(0, len(axs))]  # legends for axes
    ax_bgnds = [fig.canvas.copy_from_bbox(axs[tt].bbox) for tt in range(0, len(axs))]  # axis backgrounds
    fig.show()  # show figure
    mpu_array = np.zeros((plt_pts, 9))  # pre-allocate the 9-DoF vector
    mpu_array[mpu_array == 0] = np.nan

    # Real-Time Plot Update Loop
    ii_iter = 0  # plot update iteration counter
    cal_rot_indices = [[6, 7], [7, 8], [6, 8]]  # heading indices
    while True:
        try:
            ax, ay, az = mpu.get_accel_data()
            wx, wy, wz = mpu.get_gyro_data()
            mx, my, mz = mag.read()
        except:
            continue

        mpu_array[0:-1] = mpu_array[1:]  # get rid of last point
        mpu_array[-1] = [ax, ay, az, wx, wy, wz, mx, my, mz]  # update last point w/new data

        if ii_iter == 100:
            [fig.canvas.restore_region(ax_bgnds[tt]) for tt in range(0, len(ax_bgnds))]  # restore backgrounds
            for ii in range(0, 9):
                if ii in range(0, 3):
                    lines[ii].set_ydata(cal_offsets[ii][0] * mpu_array[:, ii] +
                                        cal_offsets[ii][1])  # update accel data array
                    axs[0].draw_artist(lines[ii])  # update accel plot
                if ii in range(3, 6):
                    lines[ii].set_ydata(np.array(mpu_array[:, ii]) - cal_offsets[ii])  # update gyro data
                    axs[1].draw_artist(lines[ii])  # update gyro plot
                if ii in range(6, 9):
                    jj = ii-6  # index offsetted to 0-2
                    x = np.array(mpu_array[:, cal_rot_indices[jj][0]])  # raw x-variable
                    y = np.array(mpu_array[:, cal_rot_indices[jj][1]])  # raw y-variable
                    x_prime = x - cal_offsets[cal_rot_indices[jj][0]]  # x-var for heading
                    y_prime = y - cal_offsets[cal_rot_indices[jj][1]]  # y-var for heading
                    x_prime[np.isnan(x)] = np.nan
                    y_prime[np.isnan(y)] = np.nan
                    # r_var = np.sqrt(np.power(x_prime, 2.0) + np.power(y_prime, 2.0))  # radius vector
                    r_var = math.hypot(x_prime, y_prime)  # radius vector
                    theta = np.arctan2(-y_prime, x_prime)  # angle vector for heading
                    lines[int(ii + jj)].set_ydata(mpu_array[:, ii] -
                                                  cal_offsets[cal_rot_indices[jj][0]])  # update mag data
                    # lines[int(ii + jj + 1)].set_data(x_prime, y_prime)  # update heading data
                    lines[int(ii + jj + 1)].set_data(theta, r_var)
                    axs[2].draw_artist(lines[int(ii + jj)])  # update mag plot
                    axs[3].draw_artist(lines[int(ii + jj + 1)])  # update heading plot
            [axs[tt].draw_artist(ax_legs[tt]) for tt in range(0, len(ax_legs))]  # update legends
            [fig.canvas.blit(axs[tt].bbox) for tt in range(0, len(ax_legs))]  # update axes
            fig.canvas.flush_events()  # flush blit events
            ii_iter = 0  # reset plot counter
        ii_iter += 1  # update plot counter


if __name__ == '__main__':
    mpu = MPU6050()  # IMU (Accelerometer, Gyroscope)
    mag = HMC5883()  # Magnetometer

    # input parameters
    mpu_labels = ['a_x', 'a_y', 'a_z',
                  'w_x', 'w_y', 'w_z',
                  'm_x', 'm_y', 'm_z']
    cal_labels = [['a_x', 'm', 'b'], ['a_y', 'm', 'b'], ['a_z', 'm', 'b'],
                  'w_x', 'w_y', 'w_z',
                  ['m_x', 'm_x0'], ['m_y', 'm_y0'], ['m_z', 'm_z0']]
    mag_cal_axes = ['z', 'y', 'x']  # axis order being rotated for mag cal
    cal_filename = 'mpu_cal_params.csv'  # filename for saving calib coeffs
    cal_size = 200  # how many points to use for calibration averages
    cal_offsets = np.array([[], [], [],
                            0.0, 0.0, 0.0,
                            [], [], []])  # cal vector
    # call to calibration functions
    re_cal_bool = input("Input 1 + Press Enter to Calibrate New Coefficients or \n" +
                        "Press Enter to Load Local Calibration Coefficients ")
    if re_cal_bool == "1":
        print("-" * 50)
        input("Press Enter to Start The Calibration Procedure")
        time.sleep(1)

        gyro_offsets = gyro_cal()  # calibrate gyro offsets under stable conditions
        cal_offsets[3:6] = gyro_offsets

        mpu_offsets = accel_cal()  # calibrate accel offsets
        cal_offsets[0:3] = mpu_offsets

        ak_offsets = mag_cal()  # calibrate mag offsets
        cal_offsets[6:] = ak_offsets

        # save calibration coefficients to file
        with open(cal_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            for param_ii in range(0, len(cal_offsets)):
                if param_ii > 2:
                    writer.writerow([cal_labels[param_ii],
                                     cal_offsets[param_ii]])
                else:
                    writer.writerow(cal_labels[param_ii] +
                                    [ii for ii in cal_offsets[param_ii]])
    else:
        # read calibration coefficients from file
        with open(cal_filename, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            iter_ii = 0
            for row in reader:
                if len(row) > 2:
                    row_vals = [float(ii) for ii in row[int((len(row)/2) + 1):]]
                    cal_offsets[iter_ii] = row_vals
                else:
                    cal_offsets[iter_ii] = float(row[1])
                iter_ii += 1

    # print out offsets for each sensor
    print("-" * 50)
    print('Offsets:')
    for param_ii, param in enumerate(cal_labels):
        print('\t{0}: {1}'.format(param, cal_offsets[param_ii]))

    # real-time plotter for validation
    print("-" * 50)
    mpu_plot_test()  # real-time plot of mpu calibratin verification
