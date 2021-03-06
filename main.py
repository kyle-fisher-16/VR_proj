import threading
import signal
import sys

# http
from urlparse import urlparse
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import json

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

# Globals
PLOT_KEYPOINTS = False
SHOW_IMAGE = True
SIGNAL_EXIT = False
CAM_RES = (640, 368)
SIFT_DOWNSAMP = 2.0;
CAM_SLEEP = 0.25;
MAIN_LOOP_SLEEP = 2.0;

accumulated_yaw_drift = 0.0;
accumulated_yaw_drift_lpf = 0.0;
current_imu_q = None;
current_imu_fused_q = None;
frame0_snapshot = None;
current_snapshot = None;


# Used for HTTP connection with JAVA client
class CustomHTTP(BaseHTTPRequestHandler):
	def _set_headers(self):
		self.send_response(200)
		self.send_header('Content-type', 'text/html')
		self.end_headers()
	def do_GET(self):
		global current_imu_fused_q, frame0_snapshot, current_snapshot, accumulated_yaw_drift
		query = urlparse(self.path).query
		query_components = dict(qc.split("=") for qc in query.split("&"))
		req_type = query_components["type"]
		to_send = ''
		
		time.sleep(0.01)
		if req_type == "imu-fused-mat":
			mat = current_imu_fused_q.inverse.rotation_matrix.reshape(-1)
			for i in range(9):
				to_send += '%f ' % mat[i]
		elif req_type == "acc-yaw-drift":
			to_send += '%f' % accumulated_yaw_drift
		elif req_type == "frame0-kp":
			for kp in frame0_snapshot.kp_normed_rot:
				to_send += '%f ' % kp[0] + '%f ' % kp[1]
		elif req_type == "current-kp":
			for kp in current_snapshot.kp_normed_rot:
				to_send += '%f ' % kp[0] + '%f ' % kp[1]
		else:
			pass

		self._set_headers()

		self.wfile.write(to_send)
	def log_message(self, format, *args):
		return

# Stores an image and IMU snapshot from one moment
class ImuCamSnapshot:
	def __init__(self, cv_kps, cv_descs, kp_points_normed, imu_fused_data, current_snapshot_rotated):
		self.cv_kp = cv_kps
		self.cv_desc = cv_descs
		self.kp_normed = kp_points_normed
		self.imu_fused_q = imu_fused_data
		self.kp_normed_rot = current_snapshot_rotated
		assert len(self.cv_desc) == len(self.cv_kp)
		assert len(self.kp_normed) == len(self.cv_kp) 

# start the http server
server_address = ('', 8000)
http_server_obj = HTTPServer(server_address, CustomHTTP)

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

# enable plotting
if PLOT_KEYPOINTS:
    import matplotlib.pyplot as plt
    plot_batch_fig, plot_batch_ax = plt.subplots(1,1)
    plt.ion()
    plt.show()

# init the camera with calibrated constants
def set_up_camera(resolution=(2592, 1536), shutter=100):
	camera = PiCamera()
	camera.resolution = resolution
	cam_matrix = np.array([[ 638.37209926,    0.,          173.93648249],
				   [   0.,          637.9986691,   328.11401563],
				   [   0.,            0.,            1.        ]]);
	cam_dists = np.array([[2.72214724e-01,  -1.76166649e+00 ,  7.50957452e-04,  2.67812475e-03,  3.21569990e+00]]);

	camera.shutter_speed = shutter_float(shutter)
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
	return q_corrected;

# convert shutter fraction to float in microsec.
def shutter_float(frac):
	return int(1000000.0 / frac)

# update interactive plot of keypoints from the camera
def update_plot(arr1, arr2=None, kp1=None, kp2=None):
	if not PLOT_KEYPOINTS:
		return;
	# draw the plot
	global plot_batch_ax, plot_batch_fig
	plot_batch_ax.clear()
	plot_batch_ax.scatter(x=arr1[:,0], y=arr1[:,1], c='g', s=2, marker='x')
	if not arr2 is None:
		plot_batch_ax.scatter(x=arr2[:,0], y=arr2[:,1], c='b', s=2, marker='+')
	if not (kp1 is None or kp2 is None):
#		plot_batch_ax.scatter(x=kp1[:,0], y=kp1[:,1], c='r', s=4, marker='+')
		for i in range(len(kp1)):
			plot_batch_ax.plot([kp1[i,0], kp2[i,0]], [kp1[i,1], kp2[i,1]])
	plt.xlim(1.0, -1.0);
	plt.ylim(-1.0, 1.0);
	#~ plt.plot([thresh, thresh], [0.0, 1.0], color='k', linestyle='-', linewidth=2)
	plot_batch_fig.canvas.draw()
	plt.pause(0.0001)

# read the imu continuously until exit
def imu_loop():
	global frame0_keypoints, current_imu_q, imu_poll_interval, accumulated_yaw_drift, current_imu_fused_q
	while not SIGNAL_EXIT:
		current_imu_q = read_imu();
		q_y_fix = pq.Quaternion(axis=[0.0, 1.0, 0.0], degrees=-accumulated_yaw_drift_lpf)
		current_imu_fused_q = q_y_fix * current_imu_q;
		time.sleep(imu_poll_interval*0.5/1000.0)

# low-pass filter for the yaw drift
lpf_ct = 0;
def drift_lpf_loop():
	global accumulated_yaw_drift, accumulated_yaw_drift_lpf,  lpf_ct
	while not SIGNAL_EXIT:
		accumulated_yaw_drift_lpf += (accumulated_yaw_drift - accumulated_yaw_drift_lpf) * 0.001;
		#print 'lpf',[accumulated_yaw_drift_lpf, accumulated_yaw_drift]
		time.sleep(0.03)
		lpf_ct += 1
		
# repeatedly take photographs
def cam_loop():
	try:
		global SIGNAL_EXIT, frame0_snapshot, current_snapshot, current_imu_fused_q
		camera, img_buffer, bgr_shape, cam_matrix, cam_dists = set_up_camera(CAM_RES, 100)
		for i, data in enumerate(camera.capture_continuous(img_buffer, burst=True, format='bgr')):
			snapshot_imu_fused_q = current_imu_fused_q
			if SIGNAL_EXIT:
				return;
			# orient the camera 
			data = data.reshape(bgr_shape)
			data = np.fliplr(np.flipud(data))
			data = np.flipud(np.flipud(data))
			gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
			gray = gray.transpose([1, 0]);
			gray_sm = cv2.resize(gray, None, fx=1.0/SIFT_DOWNSAMP, fy=1.0/SIFT_DOWNSAMP, interpolation=cv2.INTER_CUBIC)

			# detect the keypoints
			kpts, descs = sift.detectAndCompute(gray_sm, None)
			if len(kpts) == 0:
				continue;
			points_arr = np.empty((len(kpts), 1, 2), dtype="float32")
			kpt_idx = 0;
			# extract the keypoints for distortion correction
			for kp in kpts:
				kp_pt = np.array(kp.pt)
				points_arr[kpt_idx,:,:] = SIFT_DOWNSAMP*kp_pt.reshape((1,1,2))
				kpt_idx+=1;
			undistorted_pts_arr = cv2.undistortPoints(points_arr, cam_matrix, cam_dists)
			undistorted_pts_arr = undistorted_pts_arr.reshape((-1,2))

			# save this snapshot
			current_snapshot_rotated = apply_rot(undistorted_pts_arr, snapshot_imu_fused_q)
			current_snapshot = ImuCamSnapshot(kpts, descs, undistorted_pts_arr, snapshot_imu_fused_q, current_snapshot_rotated)
			if i == 0:
				frame0_snapshot = current_snapshot;

			# display the image
			if SHOW_IMAGE:
				kp_img = cv2.drawKeypoints(gray_sm, kpts, None)
				kp_img = cv2.resize(kp_img, None, fx=SIFT_DOWNSAMP, fy=SIFT_DOWNSAMP, interpolation=cv2.INTER_CUBIC)
				kp_img = np.fliplr(kp_img)
				cv2.imshow('image.jpg', kp_img)
				cv2.waitKey(1)
			time.sleep(CAM_SLEEP)
	finally:
		pass;

# apply a rotation to a set of keypoints
def apply_rot(keypoints, imu_q):
	keypoints_rot = np.empty((0, 2), dtype=np.float32)
	kp_idx = 0;
	for kp in keypoints:
		kp_homog = np.array([kp[0], -kp[1], 1.0], dtype= np.float32)
		if imu_q is None:
			kp_rot = kp_homog;
		else:
			kp_rot = imu_q.rotate(kp_homog)

		if (kp_rot[2] > 0.0):
			kp_rot = np.array([kp_rot[0] / kp_rot[2], kp_rot[1] / kp_rot[2]]).reshape((1, 2))
			keypoints_rot = np.vstack((keypoints_rot, kp_rot))
			keypoints_rot[kp_idx,:] = kp_rot;
			kp_idx += 1;
	return keypoints_rot;

# look at pinhole camera points and calculate difference in yaw
def calculate_yaw_difference(pt1, pt2):
	pt1_angle = np.degrees(math.atan2(pt1[0,0], 1.0))
	pt2_angle = np.degrees(math.atan2(pt2[0,0], 1.0))
	return pt1_angle - pt2_angle

# calcuate the best estiamte of yaw correction
def calculate_yaw_correction():
	global frame0_snapshot, current_snapshot
	bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
	matches = bf.match(current_snapshot.cv_desc, frame0_snapshot.cv_desc)
	matches_dist = map(lambda x:x.distance, matches)
	matches_sorted_idx = np.argsort(matches_dist)
	NUM_MATCHES_LIMIT = 5;

	try:
		assert len(matches) >= NUM_MATCHES_LIMIT
		kp_frame0_arr = np.empty((0, 2), dtype=np.float32)
		kp_current_arr = np.empty((0, 2), dtype=np.float32)
		matches_sorted = [matches[i] for i in matches_sorted_idx[0:NUM_MATCHES_LIMIT]]

		sum_yaw_drift = 0.0;
		num_yaw_measurements = 0;

		for match_num in range(NUM_MATCHES_LIMIT):
			frame0_snapshot_kp_idx = matches_sorted[match_num].trainIdx
			current_snapshot_kp_idx = matches_sorted[match_num].queryIdx
			kp_frame0 = frame0_snapshot.kp_normed_rot[frame0_snapshot_kp_idx].reshape(-1, 2)
			kp_current = current_snapshot.kp_normed_rot[current_snapshot_kp_idx].reshape(-1, 2)
			kp_dist = np.linalg.norm(kp_frame0 - kp_current)
		
			if kp_dist < 0.1:
				yaw_dist = calculate_yaw_difference(kp_current, kp_frame0)
				sum_yaw_drift += yaw_dist;
				num_yaw_measurements += 1;
				kp_frame0_arr = np.vstack((kp_frame0_arr, kp_frame0))
				kp_current_arr = np.vstack((kp_current_arr, kp_current))
		#print 'yaw_drift est is', 

		assert num_yaw_measurements > 0
		drift_est = sum_yaw_drift / float(num_yaw_measurements);


		return kp_frame0_arr, kp_current_arr, drift_est 
	except:
		return None, None, None
	

# repeatedly update the yaw estimation
def main_loop():
	global SIGNAL_EXIT, frame0_snapshot, current_snapshot, accumulated_yaw_drift

	while not SIGNAL_EXIT:
		# apply rotation to the keypoints so that they nearly line up
 		if frame0_snapshot is not None and \
			len(frame0_snapshot.kp_normed) > 0 and \
			len(current_snapshot.kp_normed) > 0 :
			# Get fused-rotated current imu reading
			rot1 = pq.Quaternion(axis=[0.0, 1.0, 0.0], degrees=0.0)
			kp_frame0, kp_current, drift_est = calculate_yaw_correction()
			if drift_est is not None:
				accumulated_yaw_drift += drift_est;
				update_plot(frame0_snapshot.kp_normed_rot, current_snapshot.kp_normed_rot, kp1=kp_frame0, kp2=kp_current)
		time.sleep(MAIN_LOOP_SLEEP)

# repeatedly respond to HTTP requests for data
def http_loop():
	global http_server_obj
	while not SIGNAL_EXIT:
		http_server_obj.handle_request()

# Make threads and set up ctrl-c catch
threads = []
cam_thread = threading.Thread(target=cam_loop)
imu_thread = threading.Thread(target=imu_loop)
main_thread = threading.Thread(target=main_loop)
drift_lpf_thread = threading.Thread(target=drift_lpf_loop)
http_thread = threading.Thread(target=http_loop)
threads.append(cam_thread)
threads.append(imu_thread)
threads.append(main_thread)
threads.append(drift_lpf_thread)
threads.append(http_thread)
for t in threads:
	t.start()
def signal_handler(signal, frame):
	global SIGNAL_EXIT, http_server_obj
	SIGNAL_EXIT = True
	cv2.destroyAllWindows()
	for t in threads:
		t.join()

signal.signal(signal.SIGINT, signal_handler)
signal.pause()

