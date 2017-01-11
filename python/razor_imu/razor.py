#!/usr/bin/env python

import math
import serial
import sys
import string
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler

# messages to overwrite
from sandshark_msgs.msg import OrientationEuler
from sandshark_msgs.msg import OrientationQuaternion

deg2rad = math.pi/180.0
rad2deg = 180.0/math.pi

class RazorDriver(object):

    def __init__(self, port='/dev/ttyUSB0', yaw_offset=0):
        ''' calibration parameters found using accel_gyro_calibration.py and magnetometer_calibration.py '''
        self.accel = np.array([[-268, 263], [-260, 259], [-295, 231]])
        self.gyro = np.array([-14.02, -24.60, -4.96])
        self.magnetometer_ellipsoid_center = np.array([195.724, -98.0166, -88.7335])
        self.magnetometer_ellipsoid_transform = np.array([[0.912414, 0.0349674, 0.00728801], [0.0349674, 0.953003, -0.00194013], [0.00728801, -0.00194013, 0.999365]])

        self.port = port
        self.yaw_offset = yaw_offset
        self.serial = None
        self.pub_euler = rospy.Publisher('/motion/orientation_euler', OrientationEuler, queue_size=1)
        self.pub_quaternion = rospy.Publisher('/motion/orientation_quaternion', OrientationQuaternion, queue_size=1)
        self.euler_msg = OrientationEuler()
        self.quaternion_msg = OrientationQuaternion()
        self.header_time = 0
        self.header_seq = 0
        rospy.init_node("razor_node")

    def connect(self):
        print 'Attempting to connect to Razor AHRS on port ' + self.port + '...'
        try:
            self.serial = serial.Serial(port=self.port, baudrate=57600, timeout=1)
        except serial.serialutil.SerialException:
            print 'Razor AHRS not found - set port correctly in driver.'
            sys.exit(0)
        rospy.sleep(5)
        print 'Connection established!'

    def set_calibration(self):
        print 'Setting calibration parameters...'
        #set accelerometer calibration parameters
        self.serial.write('#caxm' + str(self.accel[0,0]) + chr(13))
        self.serial.write('#caxM' + str(self.accel[0,1]) + chr(13))
        self.serial.write('#caym' + str(self.accel[1,0]) + chr(13))
        self.serial.write('#cayM' + str(self.accel[1,1]) + chr(13))
        self.serial.write('#cazm' + str(self.accel[2,0]) + chr(13))
        self.serial.write('#cazM' + str(self.accel[2,1]) + chr(13))
        #set gyroscope calibration parameters
        self.serial.write('#cgx' + str(self.gyro[0]) + chr(13))
        self.serial.write('#cgy' + str(self.gyro[1]) + chr(13))
        self.serial.write('#cgz' + str(self.gyro[2]) + chr(13))
        #set magnetometer extended calibration parameters
        self.serial.write('#ccx' + str(self.magnetometer_ellipsoid_center[0]) + chr(13))
        self.serial.write('#ccy' + str(self.magnetometer_ellipsoid_center[1]) + chr(13))
        self.serial.write('#ccz' + str(self.magnetometer_ellipsoid_center[2]) + chr(13))
        self.serial.write('#ctxX' + str(self.magnetometer_ellipsoid_transform[0,0]) + chr(13))
        self.serial.write('#ctxY' + str(self.magnetometer_ellipsoid_transform[0,1]) + chr(13))
        self.serial.write('#ctxZ' + str(self.magnetometer_ellipsoid_transform[0,2]) + chr(13))
        self.serial.write('#ctyX' + str(self.magnetometer_ellipsoid_transform[1,0]) + chr(13))
        self.serial.write('#ctyY' + str(self.magnetometer_ellipsoid_transform[1,1]) + chr(13))
        self.serial.write('#ctyZ' + str(self.magnetometer_ellipsoid_transform[1,2]) + chr(13))
        self.serial.write('#ctzX' + str(self.magnetometer_ellipsoid_transform[2,0]) + chr(13))
        self.serial.write('#ctzY' + str(self.magnetometer_ellipsoid_transform[2,1]) + chr(13))
        self.serial.write('#ctzZ' + str(self.magnetometer_ellipsoid_transform[2,2]) + chr(13))
        #print calibration values for verification
        self.serial.flushInput()
        self.serial.write('#p' + chr(13))
        calib_data = self.serial.readlines()
        calib_data_print = "Set calibration parameters:\r\n"
        for line in calib_data:
            calib_data_print += line
        print calib_data_print

    def run(self):
        # stop datastream
        self.serial.write('#o0' + chr(13))
        # flush datastream manually
        discard = self.serial.readlines()
        # set output mode (char)
        self.serial.write('#ot' + chr(13))
        # set calibration parameters
        self.set_calibration()
        # start datastream
        self.serial.write('#o1' + chr(13))
        # disable error message output
        self.serial.write('#oe0' + chr(13))
        # request sync token
        self.serial.write('#s00' + chr(13))
        print 'Discarding first 200 AHRS readings...'
        for x in range(0, 200):
            line = self.serial.readline()
        print 'Publishing AHRS data!'
        while not rospy.is_shutdown():
            line = self.serial.readline()
            line = line.replace("\r\n","")
            line = line[5:]
            yaw_pitch_roll = string.split(line,",")
            if len(yaw_pitch_roll) > 2:
                yaw = float(yaw_pitch_roll[0]) + self.yaw_offset
                yaw = -yaw  #orientation euler SandShark msg has yaw as negative of bearing
                if yaw < 0.0:
                    yaw += 360.0
                if yaw >= 360.0:
                    yaw -= 360.0
                yaw *= deg2rad
                pitch = float(yaw_pitch_roll[1])*deg2rad
                roll = float(yaw_pitch_roll[2])*deg2rad
                q = quaternion_from_euler(roll,pitch,yaw)
                self.header_time = rospy.Time.now()
                self.euler_msg.header.stamp = self.header_time
                self.euler_msg.header.seq = self.header_seq
                self.euler_msg.pitch = pitch*rad2deg
                self.euler_msg.roll = roll*rad2deg
                self.euler_msg.yaw = yaw*rad2deg
                self.pub_euler.publish(self.euler_msg)
                self.quaternion_msg.header.stamp = self.header_time
                self.quaternion_msg.header.seq = self.header_seq
                self.quaternion_msg.x = q[0]
                self.quaternion_msg.y = q[1]
                self.quaternion_msg.z = q[2]
                self.quaternion_msg.w = q[3]
                self.pub_quaternion.publish(self.quaternion_msg)
                self.header_seq += 1
        self.serial.close()

if __name__ == '__main__':
    try:
        razor = RazorDriver()
        razor.connect()
        razor.run()
    except rospy.ROSInterruptException:
        razor.serial.close()
        pass

razor.serial.close()