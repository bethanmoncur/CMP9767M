#!/usr/bin/env python

import cv2
import numpy as np
from matplotlib import pyplot as plt

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class grape_counter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.image_callback)

    def image_callback(self, data):
        # --- import the grapes image and convert to HSV ---
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	# apply blur to image
        frame = cv2.blur(cv_image, (9, 9))
	# convert BGR to HSV for filtering
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# --- filter the image by colour of the grapes ---
	# define the range of colour of the grapes in HSV
	lower_blue = np.array([70, 20, 0])
	upper_blue = np.array([235, 255, 255])
	# threshold the image to get only the colour of the grapes
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	# closing: dilation followed by erosion to close small gaps in the grape bunch
        kernel = np.ones((9,9), np.uint8)
        closing = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
	# invert the mask to use it for contouring
	mask_inv = 255 - closing

	# --- grape detection ---
	# establish minimum area for a grape bunch
	min_area = 150
	# initialise variables to be outputted
	found_grapes = False
	count = 0
	# convert from BGR to RGB for plotting
	image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
	# get the contours using the mask from colour filtering
	contours, hierarchy = cv2.findContours(mask_inv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
	cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 1)

	# --- grape bunch counting ---
	# iterate through the list of contours
	for i, contour in enumerate(contours):
	    # ignore boundary of image
	    if hierarchy[0][i][3] >= 0:
	    # if the contour is larger than the minimum area for a grape bunch
	        if cv2.contourArea(contour) >= min_area:
		    found_grapes = True
		    grape_contour = contour
		    # Get the coordinates of the grape centre
		    M = cv2.moments(grape_contour)
		    cx = int(M['m10'] / M['m00'])
		    cy = int(M['m01'] / M['m00'])
		    count += 1
		    # Plot the grape centre point
		    cv2.circle(image_rgb, (cx, cy), 4, (255, 0, 0), -1)
        result_statement = "Number of grape bunches: " + str(count)
	print("Grapes detected: " + str(found_grapes))
	print(result_statement)

	# uncomment to display the colour mask, contours and detected grape bunches
        cv2.imshow("Colour thresholding", closing)
        cv2.imshow("Contours", cv_image)
        cv2.imshow("Results", image_rgb)
	cv2.waitKey(0)


# cv2.startWindowThread()
rospy.init_node('grape_counter')
gc = grape_counter()
rospy.spin()		
cv2.destroyAllWindows()


