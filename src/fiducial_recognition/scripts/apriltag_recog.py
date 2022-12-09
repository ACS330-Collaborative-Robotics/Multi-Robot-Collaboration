
#import files
import numpy as np
import cv2
import time 
import argparse
import apriltag

#open camera and aruco detection
cap = cv2.VideoCapture(0) # assigning camera (devices webcam) to variable

#reading/detecting for apriltags while camera is open
while True:
	ret, img = cap.read()
	h, w, _ = img.shape
	width = 1000
	height = int(width*(h/w))
	img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	options = apriltag.DetectorOptions(families="tag36h11") #define apriltags type 
	detector = apriltag.Detector(options)
	results = detector.detect(gray) #detect apriltags
	print("[INFO] {} total AprilTags detected".format(len(results)))
	

#draw box around apriltags and a red dot in the middle 
	for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
		(ptA, ptB, ptC, ptD) = r.corners
		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptD[1]))
		ptA = (int(ptA[0]), int(ptA[1]))
	# draw the bounding box of the AprilTag detection
		cv2.line(img, ptA, ptB, (0, 255, 0), 2)
		cv2.line(img, ptB, ptC, (0, 255, 0), 2)
		cv2.line(img, ptC, ptD, (0, 255, 0), 2)
		cv2.line(img, ptD, ptA, (0, 255, 0), 2)
	# draw the center (x, y)-coordinates of the AprilTag
		(cX, cY) = (int(r.center[0]), int(r.center[1]))
		cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
	# draw the tag family on the image
		tagFamily = r.tag_family.decode("utf-8")
		cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		print("[INFO] tag family: {}".format(tagFamily))

	if cv2.waitKey(1) == ord('q'): # quit the camera view when 'q' is pressed
		break
	cv2.imshow("Image", img)


cap.release()
cv2.destroyAllWindows()
	






