#!/usr/bin/env python

import math
import serial
import time
import sys
import string
import numpy as np

deg2rad = math.pi/180.0
rad2deg = 180.0/math.pi

class RazorSetCalibValues(object):

    def __init__(self, port='/dev/ttyUSB0'):
        # calibration parameters found using accel_gyro_calibration.py and magnetometer_calibration.py
        self.accel = np.array([[-250, 250], [-250, 250], [-250, 250]])
        self.gyro = np.array([0, 0, 0])
        self.magnetometer_ellipsoid_center = np.array([0, 0, 0])
        self.magnetometer_ellipsoid_transform = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

        self.port = port
        self.serial = None

    def connect(self):
        print 'Attempting to connect to Razor AHRS on port ' + self.port + '...'
        try:
            self.serial = serial.Serial(port=self.port, baudrate=57600, timeout=1)
        except serial.serialutil.SerialException:
            print 'Razor AHRS not found - set port correctly in driver.'
            sys.exit(0)
        time.sleep(5)
        print 'Connection established!'

    def run(self):
        # stop datastream
        self.serial.write('#o0' + chr(13))
        # flush datastream manually
        discard = self.serial.readlines()
        # disable error message output
        self.serial.write('#oe0' + chr(13))

        #self.serial.write('#caxm' + str(-300) + chr(13))

        # request to print calibration values
        self.serial.write('#p' + chr(13))
        try:
            while True:
                line = self.serial.readline()
                print line[0:-2]
                if line[0:21] == 'GYRO_AVERAGE_OFFSET_Z':
                    break
        except KeyboardInterrupt:
                pass
        self.serial.close()

if __name__ == '__main__':
    razor = RazorSetCalibValues()
    razor.connect()
    razor.run()

razor.serial.close()