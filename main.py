import threading
import signal
import sys

# camera
from picamera import PiCamera
import time
import numpy as np
import cv2
import io
import csv

# imu
import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import operator
import numpy as np
import pyquaternion as pq

# plot
import matplotlib.pyplot as plt

# Globals
PLOT_KEYPOINTS = True
SIGNAL_EXIT = False
CAM_RES = (640, 368)
SIFT_DOWNSAMP = 2.0;
frame0_keypoints = None;
frame0_imu_q = None;

current_imu_q = None;
snapshot_imu_q = None;
snapshot_keypoints = None;

# initialize the IMU
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
imu_poll_interval = imu.IMUGetPollInterval()

# init sift
sift = cv2.xfeatures2d.SIFT_create()

if PLOT_KEYPOINTS:
    plot_batch_fig, plot_batch_ax = plt.subplots(1,1)
    plt.ion()
    plt.show()

def set_up_camera(resolution=(2592, 1536), shutter=100):
	camera = PiCamera()
	camera.resolution = resolution
	'''cam_matrix = np.array([[ 627.87318394 ,   0.      ,    313.24254997],
							[   0.  ,        629.02389272  ,194.73906385], 
							[   0.  ,          0.    ,        1.        ]])
	cam_dists = np.array([[  2.54539197e-01 , -1.56778301e+00  ,-1.36953218e-03 , -1.15023962e-03, 2.68181752e+00]])'''
	cam_matrix = np.array([[ 638.37209926,    0.,          173.93648249],
				   [   0.,          637.9986691,   328.11401563],
				   [   0.,            0.,            1.        ]]);
	cam_dists = np.array([[2.72214724e-01,  -1.76166649e+00 ,  7.50957452e-04,  2.67812475e-03,  3.21569990e+00]]);

	#~ camera.shutter_speed = shutter_float(shutter)
	#~ camera.iso = 2000
	#~ camera.iso = 
	print 'Starting camera...'
	time.sleep(2)
	print 'Done!'
	
	bgr_shape = (resolution[1], resolution[0], 3)
	img_buffer = np.empty(bgr_shape, dtype=np.uint8).reshape(-1)
	return camera, img_buffer, bgr_shape, cam_matrix, cam_dists

def read_imu():
	global imu, current_imu_q
	
	# wait for the first reading
	while not imu.IMURead():
		pass
	data = imu.getIMUData()
		
	# Process quaternion
	q = np.asarray(data["fusionQPose"])
	q_raw = pq.Quaternion(q[0], q[1], q[2], q[3])
	q_x_fix = pq.Quaternion(axis=[1.0, 0.0, 0.0], degrees=-90.0)
	q_corrected = q_x_fix * q_raw
	q_corrected[2] *= -1.0;
	q_corrected[3] *= -1.0; 
	
#	print 'q angle, axis', np.around(q_corrected.axis,2), np.around(np.degrees(q_corrected.angle), 2)

	return q_corrected;

def shutter_float(frac):
	return int(1000000.0 / frac)
	
def update_plot(arr1, arr2=None):
	if not PLOT_KEYPOINTS:
		return;
	# draw the plot
	global plot_batch_ax, plot_batch_fig
	plot_batch_ax.clear()
	plot_batch_ax.scatter(x=arr1[:,0], y=arr1[:,1], c='g', s=2, marker='x')
	if not arr2 is None:
		plot_batch_ax.scatter(x=arr2[:,0], y=arr2[:,1], c='b', s=2, marker='+')
	plt.xlim(1.0, -1.0);
	plt.ylim(-1.0, 1.0);
	#~ plt.plot([thresh, thresh], [0.0, 1.0], color='k', linestyle='-', linewidth=2)
	plot_batch_fig.canvas.draw()
	plt.pause(0.0001)

def imu_loop():
	global frame0_keypoints, current_imu_q, imu_poll_interval
	while not SIGNAL_EXIT:
		current_imu_q = read_imu();
		time.sleep(imu_poll_interval*0.5/1000.0)
 

def cam_loop():
	try:
		global SIGNAL_EXIT, frame0_keypoints, frame0_imu_q, snapshot_imu_q, snapshot_keypoints
		camera, img_buffer, bgr_shape, cam_matrix, cam_dists = set_up_camera(CAM_RES, 100)
		for i, data in enumerate(camera.capture_continuous(img_buffer, burst=True, format='bgr')):
			snapshot_imu_q = current_imu_q;
			if SIGNAL_EXIT:
				return;
			# DETECT KEYPOINTS
#			print 'frame', i
			data = data.reshape(bgr_shape)
			data = np.fliplr(np.flipud(data))
			data = np.flipud(np.flipud(data))

			gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
			gray = gray.transpose([1, 0]);
			gray_sm = cv2.resize(gray, None, fx=1.0/SIFT_DOWNSAMP, fy=1.0/SIFT_DOWNSAMP, interpolation=cv2.INTER_CUBIC)
		
			kpts = sift.detect(gray_sm, None)
		
			if len(kpts) == 0:
				continue;
		
			points_arr = np.empty((len(kpts), 1, 2), dtype="float32")
			kpt_idx = 0;

			for kp in kpts:
				kp_pt = np.array(kp.pt)
				points_arr[kpt_idx,:,:] = SIFT_DOWNSAMP*kp_pt.reshape((1,1,2))
				kpt_idx+=1;

			# undistort
			undistorted_pts_arr = cv2.undistortPoints(points_arr, cam_matrix, cam_dists)
			undistorted_pts_arr = undistorted_pts_arr.reshape((-1,2))
		
		
			# show image	
			kp_img = cv2.drawKeypoints(gray_sm, kpts, None)
			kp_img = cv2.resize(kp_img, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
			if i==0:
				cv2.imwrite("image.jpg", kp_img)
			'''
			cv2.imshow('image.jpg', kp_img)
			cv2.waitKey(1)
			break;
			'''

			snapshot_keypoints = undistorted_pts_arr;
			if i == 0:
				frame0_keypoints = snapshot_keypoints
				frame0_imu_q = snapshot_imu_q
	finally:
	    camera.close()
	    cv2.destroyAllWindows()

def apply_rot(keypoints, imu_q):

	print 'mainloop'
	keypoints_rot = np.empty(keypoints.shape, dtype=np.float32)
	kp_idx = 0;
	for kp in keypoints:
		kp_homog = np.array([kp[0], -kp[1], 1.0], dtype=np.float32)
		if imu_q is None:
			kp_rot = kp_homog;
		else:
			kp_rot = imu_q.rotate(kp_homog)
		kp_rot = np.array([kp_rot[0] / kp_rot[2], kp_rot[1] / kp_rot[2]])
		keypoints_rot[kp_idx,:] = kp_rot;
		kp_idx += 1;
	return keypoints_rot;

def main_loop():
	global frame0_keypoints, frame0_imu_q , current_imu_q, snapshot_imu_q, snapshot_keypoints
#	frame0_keypoints = np.array([[0.0, 0.0]], dtype=np.float32)
	while not SIGNAL_EXIT:
		# apply rotation to the keypoints so that they nearly line up
 		if frame0_keypoints != None and frame0_imu_q != None:
			frame0_keypoints_rot = apply_rot(frame0_keypoints, frame0_imu_q)
			current_keypoints_rot = apply_rot(snapshot_keypoints, snapshot_imu_q)
			update_plot(frame0_keypoints_rot, current_keypoints_rot)
			

# Make threads and set up ctrl-c catch
threads = []
cam_thread = threading.Thread(target=cam_loop)
imu_thread = threading.Thread(target=imu_loop)
main_thread = threading.Thread(target=main_loop)
threads.append(cam_thread)
threads.append(imu_thread)
threads.append(main_thread)
for t in threads:
	t.start()
def signal_handler(signal, frame):
	global SIGNAL_EXIT
	SIGNAL_EXIT = True
	for t in threads:
		t.join()
signal.signal(signal.SIGINT, signal_handler)
signal.pause()










'''# write points to csv file
with open('undistort_pts.csv', 'wb') as csvfile:
	data_writer = csv.writer(csvfile, delimiter=',')
	data_writer.writerows(undistorted_pts_arr)'''


