#!/usr/bin/env python

import math
import serial
import time
import sys
import string
import re

deg2rad = math.pi/180.0
rad2deg = 180.0/math.pi

class RazorAccelGyroCalib(object):

    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.serial = None
        self.first_run = True

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
        if self.first_run:
            # stop datastream
            self.serial.write('#o0' + chr(13))
            # flush datastream manually
            discard = self.serial.readlines()
            # set output mode (char)
            self.serial.write('#ot' + chr(13))
            # start datastream
            self.serial.write('#o1' + chr(13))
            # disable error message output
            self.serial.write('#oe0' + chr(13))
            # request calibration
            self.serial.write('#oc' + chr(13))
            self.first_run = False
        else:
            # request calibration
            self.serial.write('#oc' + chr(13))
        try:
            while True:
                line = self.serial.readline()
                if line[0:4] == 'magn': #skip magnetometer - we use extended magnetometer calibration with magnetometer_calibration.py
                    self.serial.write('#on' + chr(13))
                    # request calibration
                    self.serial.write('#oc' + chr(13))
                print line
        except KeyboardInterrupt:
            print 'Input: r to reset, n to move to next sensor, any other key to exit.'
            mode=raw_input('Input:')
            if (mode == 'r'):
                # request calibration
                self.serial.write('#oc' + chr(13))
                self.run()
            elif(mode == 'n'):
                # go to next mode (accel->magn->gyro (skip magn))
                self.serial.write('#on' + chr(13))
                # request calibration
                self.serial.write('#oc' + chr(13))
                self.run()
            else:
                pass
        self.serial.close()

if __name__ == '__main__':
    razor = RazorAccelGyroCalib()
    razor.connect()
    razor.run()

razor.serial.close()