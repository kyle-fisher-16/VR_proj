from picamera import PiCamera
import time
import numpy as np
import cv2
import io

objp = np.zeros((4*4, 3), np.float32)
objp[:,:2] = np.mgrid[0:4,0:4].T.reshape(-1,2)

objpoints=[]
imgpoints=[]
img_shape = None

for i in range(100):
	filename =  "imgs/"+str(i) +".png";
	img = cv2.imread(filename)
	
	if (img != None):
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		img_shape = img.shape
		ret, corners = cv2.findChessboardCorners(img, (4, 4), None)
		imgpoints.append(corners)
		objpoints.append(objp)
		
		print i
		
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape[::-1], None, None)
print 'ret', ret
print 'mtx', mtx
print 'dist', dist
print 'rvecs', rvecs
print 'tvecs', tvecs

quit()
	#~ 
#~ 
#~ ret, corners = cv2.findChessboardCorners(gray, (4, 4), None)
		#~ if ret == True:
#~ 
			#~ cv2.drawChessboardCorners(gray, (4, 4), corners, ret)
			#~ calib_img_num+=1;
			#~ time.sleep(1)
		#~ else:
			#~ print 'No chessboard found'
			#~ 
		#~ cv2.imshow('image.jpg', gray)
		#~ cv2.waitKey(1)
		#~ 
#~ finally:
    #~ camera.close()
    #~ cv2.destroyAllWindows()
