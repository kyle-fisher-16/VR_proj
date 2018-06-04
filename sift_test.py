from picamera import PiCamera
import time
import numpy as np
import cv2
import io
import csv



# initialize camera
print('setting camera parameters...')


#~ CAM_RES = (2592//3, 1536//3)
CAM_RES = (640, 368)
SIFT_DOWNSAMP = 2.0;

sift = cv2.xfeatures2d.SIFT_create()



def set_up_camera(resolution=(2592, 1536), shutter=100):
	camera = PiCamera()
	camera.resolution = resolution
	cam_matrix = np.array([[ 627.87318394 ,   0.      ,    313.24254997],
							[   0.  ,        629.02389272  ,194.73906385], 
							[   0.  ,          0.    ,        1.        ]])
	cam_dists = np.array([[  2.54539197e-01 , -1.56778301e+00  ,-1.36953218e-03 , -1.15023962e-03, 2.68181752e+00]])
	#~ camera.shutter_speed = shutter_float(shutter)
	#~ camera.iso = 2000
	#~ camera.iso = 
	print 'Starting camera...'
	time.sleep(2)
	print 'Done!'
	
	bgr_shape = (resolution[1], resolution[0], 3)
	img_buffer = np.empty(bgr_shape, dtype=np.uint8).reshape(-1)
	return camera, img_buffer, bgr_shape, cam_matrix, cam_dists

def shutter_float(frac):
	return int(1000000.0 / frac)

try:
	camera, img_buffer, bgr_shape, cam_matrix, cam_dists = set_up_camera(CAM_RES, 100)
	for i, data in enumerate(camera.capture_continuous(img_buffer, burst=True, format='bgr')):
		print 'frame', i
		data = data.reshape(bgr_shape)
		data = np.fliplr(np.flipud(data))
		data = np.flipud(np.flipud(data))

		gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
		gray = gray.transpose([1, 0]);
		gray = cv2.resize(gray, None, fx=1.0/SIFT_DOWNSAMP, fy=1.0/SIFT_DOWNSAMP, interpolation=cv2.INTER_CUBIC)
		
		kpts = sift.detect(gray, None)
		
		points_arr = np.empty((len(kpts), 1, 2), dtype="float32")
		kpt_idx = 0;
		for kp in kpts:
			kp_pt = np.array(kp.pt)
			points_arr[kpt_idx,:,:] = kp_pt.reshape((1,1,2))
			kpt_idx+=1;
		
		undistorted_pts_arr = cv2.undistortPoints(points_arr, cam_matrix, cam_dists)
		undistorted_pts_arr = undistorted_pts_arr.reshape((-1,2))
		print len(undistorted_pts_arr),'keypoints detected'
		
		
		
		#~ # write points to csv file
		#~ with open('undistort_pts.csv', 'wb') as csvfile:
			#~ data_writer = csv.writer(csvfile, delimiter=',')
			#~ data_writer.writerows(undistorted_pts_arr)
		#~ 
		#~ 
		#~ # show image	
		#~ kp_img = cv2.drawKeypoints(gray, kpts, None)
		#~ kp_img = cv2.resize(kp_img, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
		#~ cv2.imwrite("image.jpg", kp_img)
		#~ cv2.imshow('image.jpg', kp_img)
		#~ cv2.waitKey(1)
		

finally:
    camera.close()
    cv2.destroyAllWindows()
