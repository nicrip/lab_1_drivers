#!/usr/bin/env python

import math
import serial
import time
import sys
import string
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

deg2rad = math.pi/180.0
rad2deg = 180.0/math.pi

class RazorMagnetometerCalibration(object):

    def __init__(self, port='/dev/ttyUSB0', num_readings=3000, display=True):
        self.port = port
        self.num_readings = num_readings
        self.display = display
        self.serial = None
        self.readings = np.zeros([self.num_readings,3])

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
        # set output mode (raw sensor char)
        self.serial.write('#osrt' + chr(13))
        # start datastream
        self.serial.write('#o1' + chr(13))
        # disable error message output
        self.serial.write('#oe0' + chr(13))
        # request sync token
        self.serial.write('#s00' + chr(13))
        print 'Magnetometer readings starting in 5s...'
        time.sleep(1)
        print 'Magnetometer readings starting in 4s...'
        time.sleep(1)
        print 'Magnetometer readings starting in 3s...'
        time.sleep(1)
        print 'Magnetometer readings starting in 2s...'
        time.sleep(1)
        print 'Magnetometer readings starting in 1s...'
        time.sleep(1)
        print 'Magnetometer readings started!'
        print 'Discarding first 200 raw readings...'
        for x in range(0, 200):
            line = self.serial.readline()
        i = 0
        try:
            while i < self.num_readings:
                line = self.serial.readline()
                if line[0:5] == "#M-R=":
                    if (self.num_readings-i)%100 == 0:
                        print str(self.num_readings-i) + " readings remain..." 
                    line = line.replace("\r\n","")
                    line = line[5:]
                    mag_x_y_z = string.split(line,",")
                    self.readings[i,0] = float(mag_x_y_z[0])
                    self.readings[i,1] = float(mag_x_y_z[1])
                    self.readings[i,2] = float(mag_x_y_z[2])
                    i += 1
        except KeyboardInterrupt:
            pass
        print "Magnetometer readings finished - performing ellipsoid fit calibration..."

        transform, translate = self.calibrate()

        print "Ellipsoid fitting done! Writing raw values to file..."
        with open('magnetometer.csv', 'wb') as csvfile:
            mag_writer = csv.writer(csvfile, delimiter=',')
            for i in range(0, self.readings.shape[0]):
                mag_writer.writerow([self.readings[i,0], self.readings[i,1], self.readings[i,2]])

        print "Magnetometer calibration values:"
        print ""
        print "In the Razor_AHRS.ino, under 'SENSOR CALIBRATION' find the section that reads 'Magnetometer (extended calibration)"
        print "Replace the existing 3 lines with these:"
        print "#define CALIBRATION__MAGN_USE_EXTENDED true"
        print "const float magn_ellipsoid_center[3] = {%.6g,"%translate[0,0] + " %.6g,"%translate[0,1] + " %.6g};"%translate[0,2]
        print "const float magn_ellipsoid_transform[3][3] = {{%.6g,"%transform[0,0] + " %.6g,"%transform[0,1] + " %.6g},"%transform[0,2] + " {%.6g,"%transform[1,0] + " %.6g,"%transform[1,1] + " %.6g},"%transform[1,2] + " {%.6g,"%transform[2,0] + " %.6g,"%transform[2,1] + " %.6g}};"%transform[2,2]
        print ""
        print "In the razor.py and iPyRazor.py, replace self.magnetometer_ellipsoid_center and self.magnetometer_ellipsoid_transform with:"
        print "self.magnetometer_ellipsoid_center = np.array([%.6g,"%translate[0,0] + " %.6g,"%translate[0,1] + " %.6g])"%translate[0,2]
        print "self.magnetometer_ellipsoid_transform = np.array([[%.6g,"%transform[0,0] + " %.6g,"%transform[0,1] + " %.6g],"%transform[0,2] + " [%.6g,"%transform[1,0] + " %.6g,"%transform[1,1] + " %.6g],"%transform[1,2] + " [%.6g,"%transform[2,0] + " %.6g,"%transform[2,1] + " %.6g]])"%transform[2,2]
        print ""
        print "Magnetometer calibration Done!"

        if self.display:
            transform_readings = self.readings - translate
            transform_readings = np.dot(transform_readings, transform)

            fig = plt.figure()
            ax = fig.add_subplot(121, projection='3d')
            ax.scatter(self.readings[:,0], self.readings[:,1], self.readings[:,2])
            plt.title('Original Mag Readings')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax1 = fig.add_subplot(122, projection='3d')
            ax1.scatter(transform_readings[:,0], transform_readings[:,1], transform_readings[:,2], c='r')
            plt.title('Transformed Mag Readings')
            ax1.set_xlabel('X')
            ax1.set_ylabel('Y')
            ax1.set_zlabel('Z')
            plt.show()

        self.serial.close()

    def calibrate(self):
        D = np.zeros([self.readings.shape[0],9])
        D[:,0] = self.readings[:,0]*self.readings[:,0]
        D[:,1] = self.readings[:,1]*self.readings[:,1]
        D[:,2] = self.readings[:,2]*self.readings[:,2]
        D[:,3] = self.readings[:,0]*self.readings[:,1]*2
        D[:,4] = self.readings[:,0]*self.readings[:,2]*2
        D[:,5] = self.readings[:,1]*self.readings[:,2]*2
        D[:,6] = self.readings[:,0]*2
        D[:,7] = self.readings[:,1]*2
        D[:,8] = self.readings[:,2]*2
        tempA = np.dot(D.T, D)
        tempB = np.dot(D.T, np.ones([self.readings.shape[0],1]))
        v = np.linalg.solve(tempA, tempB)
        A = np.array([[v[0,0], v[3,0], v[4,0], v[6,0]], [v[3,0], v[1,0], v[5,0], v[7,0]], [v[4,0], v[5,0], v[2,0], v[8,0]], [v[6,0], v[7,0], v[8,0], -1.0]])
        center = np.linalg.solve(-A[0:3, 0:3], np.array([[v[6,0]], [v[7,0]], [v[8,0]]]))
        T = np.eye(4)
        T[3, 0:3] = center.T
        R = reduce(np.dot, [T, A, T.T])
        eigvals, eigvecs = np.linalg.eig(R[0:3, 0:3]/(-R[3,3]))
        radii = np.sqrt(1.0/eigvals.real)
        scale = np.zeros([3,3])
        scale[0,0] = radii[0]
        scale[1,1] = radii[1]
        scale[2,2] = radii[2]
        scale = np.min(radii)*np.linalg.inv(scale)
        comp = reduce(np.dot, [eigvecs, scale, eigvecs.T])
        return comp, center.T

if __name__ == '__main__':
    razor = RazorMagnetometerCalibration()
    razor.connect()
    razor.run()

razor.serial.close()