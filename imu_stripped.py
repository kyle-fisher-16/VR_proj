import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import operator
import numpy as np
import pyquaternion as pq

SETTINGS_FILE = "RTIMULib"
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

if (not imu.IMUInit()):
	print 'Failed to init IMU'
	sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(False)

poll_interval = imu.IMUGetPollInterval()

# magnetic deviation

while True:

  if imu.IMURead():
    data = imu.getIMUData()
    #~ print data
    
    q = np.asarray(data["fusionQPose"])
    #~ print q
    
    #~ print data.keys()
    time.sleep(poll_interval*1.0/1000.0)
    
    q_raw = pq.Quaternion(q[0], q[1], q[2], q[3])
    q_x_fix = pq.Quaternion(axis=[1.0, 0.0, 0.0], degrees=-90.0)
    
    q_corrected = q_x_fix * q_raw
    #~ q_corrected[1] *= -1.0;
    q_corrected[2] *= -1.0;
    q_corrected[3] *= -1.0; 
    #~ look_dir = np.array([0.0, 0.0, 1.0])
    #~ 
    #~ look_dir = q_corrected.rotate(look_dir);
    
    print 'axis:', q_corrected.axis
    print 'angle:', np.degrees(q_corrected.angle)
    
