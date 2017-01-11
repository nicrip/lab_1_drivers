import serial
import lcm
import time
import math

import sys
sys.path.insert(0, '/home/ubuntu/njord/embedded/software/build/njord_lcmtypes/lcmtypes/python/njord')
from servogroup_set_t import *

#ser = serial.Serial('/dev/ttyS0', 9600)
#ser.write("hello\n");
#ser.close()

lc = lcm.LCM()
msg = servogroup_set_t()

fun = True

while (True):
  msg.utime = msg.utime+1
  input = 500*math.sin(2*3.14159265358979323*0.1*msg.utime)+3000
  print input, fun
  msg.servo_setpoints = (input,input,input,input,input,input)
  lc.publish("NJORD_V2V_VEH1_SET_SERVOGROUP", msg.encode())
  time.sleep(0.05)

print 'something'
