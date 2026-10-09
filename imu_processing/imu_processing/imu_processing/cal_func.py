#!/usr/bin/env python3

# Python imports
import math


class CalibrateFuncs:
    def open_calbias_file(calibration_path):                    # INPUT FILE PATH TO BIAS FILE
        with open(calibration_path, 'r') as f:
            bias_values = [float(line.strip()) for line in f]
        return bias_values

    def cal_gyro(gyro_raw, gyro_bias):
        # Calibrate the gyro data by subtracting the bias from the raw data
        gyro_cal = gyro_raw - gyro_bias
        return gyro_cal