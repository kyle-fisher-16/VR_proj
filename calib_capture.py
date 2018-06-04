from picamera import PiCamera
import time
import numpy as np
import cv2
import io

# initialize camera
print('setting camera parameters...')


#~ CAM_RES = (2592//3, 1536//3)
CAM_RES = (640, 368)

sift = cv2.xfeatures2d.SIFT_create()

def set_up_camera(resolution=(2592, 1536), shutter=100):
	camera = PiCamera()
	camera.resolution = resolution
	#~ camera.shutter_speed = shutter_float(shutter)
	#~ camera.iso = 2000
	#~ camera.iso = 
	print 'Starting camera...'
	time.sleep(2)
	print 'Done!'
	
	bgr_shape = (resolution[1], resolution[0], 3)
	img_buffer = np.empty(bgr_shape, dtype=np.uint8).reshape(-1)
	return camera, img_buffer, bgr_shape

def shutter_float(frac):
	return int(1000000.0 / frac)

try:
	camera, img_buffer, bgr_shape = set_up_camera(CAM_RES, 100)
	calib_img_num = 1;
	for i, data in enumerate(camera.capture_continuous(img_buffer, burst=True, format='bgr')):
		print 'frame', i
		data = data.reshape(bgr_shape)
		data = np.fliplr(np.flipud(data))
		gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
		gray = cv2.resize(gray, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
		
		ret, corners = cv2.findChessboardCorners(gray, (4, 4), None)
		if ret == True:
			cv2.imwrite("imgs/"+str(calib_img_num)+".png", gray)
			cv2.drawChessboardCorners(gray, (4, 4), corners, ret)
			calib_img_num+=1;
			time.sleep(1)
		else:
			print 'No chessboard found'
			
		cv2.imshow('image.jpg', gray)
		cv2.waitKey(1)
		
finally:
    camera.close()
    cv2.destroyAllWindows()
