#!/usr/bin/env python

# import packages
import cv2
import numpy as np
import sys, time
import rospy, roslib, image_geometry, tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PoseStamped, Point32, PointStamped
from std_msgs.msg import String, Int32, Header
from cv_bridge import CvBridge, CvBridgeError


class grape_counter:

    def __init__(self):
        # subscribe to commands from the control node
        self.control_sub = rospy.Subscriber('counting_control', String, self.control_callback)
        # publish feedback to the control node 
        self.counting_status_pub = rospy.Publisher('counting_status', String, queue_size=10)

        # subscribe to images and information from the rgb and depth cameras
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.count_grapes)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_sub = rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect", Image, self.image_depth_callback)
        # publish images and a point cloud to visualise the image processing and grape counting steps
        self.grape_location_pub = rospy.Publisher('/thorvald_001/grape_location', Image, queue_size=10)
        self.grape_contours_pub = rospy.Publisher('/thorvald_001/grape_contours', Image, queue_size=10)
        self.point_pub = rospy.Publisher('/thorvald_001/grape_visualisation', PointCloud, queue_size=10, latch='true')

        # publish the number of grapes counted
        self.grape_count_pub = rospy.Publisher('grape_count', Int32, queue_size=30)

        # create objects for image processing, coordinate transformations and point cloud visualisations
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.point_cloud = PointCloud()

        # initialise variables 
        self.camera_model = None
        self.image_depth = None
        self.start_counting = False
        self.reset_counting = False
        self.count = 0
        self.previous_count = 0
        self.found_grapes = False
        self.counted_grape_coords = []


    # function to get commands from the control node to begin, pause or reset counting
    def control_callback(self, data):
        print str(data)
        if 'start count' in str(data):
            self.start_counting = True
        else:
            self.start_counting = False
        
        if 'reset count' in str(data):
            self.reset_counting = True


    # function to obtain the camera information 
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once


    # function to obtain the depth information
    def image_depth_callback(self, data):
        self.image_depth = data


    # function to perform colour thresholding on the image and return contours from contour detection
    def image_processing(self, image):
        # apply blur to bgr image
        blurred = cv2.blur(image, (3, 3))
	# convert BGR to HSV for filtering
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# define the range of colour of the grapes in HSV
	lower_blue = np.array([70, 20, 0])
	upper_blue = np.array([235, 255, 255])
	# threshold the image to get only the colour of the grapes
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	# closing: dilation followed by erosion to close small gaps in the grape bunch
        kernel = np.ones((7,7), np.uint8)
        closing = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
        # publish the image following closing to view in rviz
        self.grape_contours_pub.publish(self.bridge.cv2_to_imgmsg(closing))
	# invert the mask to use it for contouring
	mask_inv = 255 - closing
	# contour detection
	contours, hierarchy = cv2.findContours(mask_inv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        # draw the contours, used during testing
	cv2.drawContours(closing, contours, -1, (0, 0, 255), 1)
        return [contours, hierarchy]
    

    # function to get the position and depth of the centroid of the grape contour in terms of image coordinates 
    def get_centroid(self, grape_contour, image, depth):
        # get the coordinates of the centre of the contour    
        M = cv2.moments(grape_contour)
	cx = int(M['m10'] / M['m00'])
	cy = int(M['m01'] / M['m00'])
	image_coords = (cy, cx)
	# plot the grape centre point to visualise in rviz
	cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        # publish the grape location so it can be viewed in rviz
        self.grape_location_pub.publish(self.bridge.cv2_to_imgmsg(image))
		    
	# find the depth of the grape centroid
	# ratio between cameras calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file, 
        # sourced from lecture tutorial image_projection3
	colour_depth_ratio = (84.1/1920) / (70.0/512)
	# find the corresponding pixels in the depth image using .shape which returns rows (y), columns (x)
	depth_coords = (depth.shape[0]/2 + (image_coords[0] - image.shape[0]/2)*colour_depth_ratio, depth.shape[1]/2 + (image_coords[1] - image.shape[1]/2)*colour_depth_ratio)
	# depth has shape (424, 512), check boundary
	depth_x = int(depth_coords[1])
	depth_y = int(depth_coords[0])
	if depth_x >= 512 or depth_y >= 424:
	    return None
	# retrieve the depth reading at the centroid location 
        depth_value = depth[depth_y, depth_x]
        # if depth of the centroid of the grape not detected, skip contour
        if np.isnan(depth_value):
            return None
                        
        #project the image coords (x,y) into 3D ray in camera coords
        camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))
        # normalize the vector by z (camera_coords[2]) and then multiply by depth  
        camera_coords = [(i*depth_value)/camera_coords[2] for i in camera_coords]
        return camera_coords
    
        
    # function to transform the grape centroid from camera to map coordinates using tf and return its position    
    def image_world_tf(self, time, coords):
        # compose the PointStamped message consisting of the camera coordinates
        grape_location = PointStamped()
        grape_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"
        grape_location.header.stamp = time
        grape_location.point.x = coords[0]
        grape_location.point.y = coords[1]
        grape_location.point.z = coords[2]
        # wait for the transform to be found between the map and the camera     
        self.tf_listener.waitForTransform('map', 'thorvald_001/kinect2_right_rgb_optical_frame', time, rospy.Duration(4.0))
        # transform the camera coordinates into map coordinates
        map_coords =  self.tf_listener.transformPoint('map', grape_location)
        return map_coords


    # function to publish the grape centroid to display in rviz as a point cloud
    def point_cloud_projection(self, time, centroid):
        # compose the PointCloud message consisting of the map coordinates
        header = Header()
        header.stamp = time
        header.frame_id = 'map'
        self.point_cloud.header = header
        self.point_cloud.points.append(Point32(centroid.point.x, centroid.point.y, centroid.point.z))
        # publish the PointCloud message
        self.point_pub.publish(self.point_cloud)


    # function to reset the grape count and publish the number of grape bunches to the control node
    def reset_count(self):
        # publish the grape count
        self.grape_count_pub.publish(self.count)
        # reinitialise variables
        self.count = 0
        self.counted_grape_coords = []
        self.point_cloud = PointCloud()
        self.reset_counting = False
        # publish the command to resume navigation
        self.counting_status_pub.publish('counting complete')
    

    # function to count the number of grapes in the image and avoid double counting (uses the above functions)
    def count_grapes(self, data):
        # wait for the camera information to arrive
        if self.camera_model is None:
            return
        # wait for the depth image to arrive
        if self.image_depth is None:
            return

        # begin counting only if the control node has published the command to start counting
        if self.start_counting:

            # save the previous count to compare against the current count
            self.previous_count = self.count
            
            # get the time to enable transforms between the image frame and the map
            image_time = rospy.Time.now()
            self.tf_listener.waitForTransform('map', 'thorvald_001/kinect2_right_rgb_optical_frame', image_time, rospy.Duration(4.0))
            
            # import the grapes image and depth image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(self.image_depth, "32FC1")
            
            # perform colour thresholding and contour detection on the rgb image           
            contours = self.image_processing(image=cv_image)[0]
            hierarchy = self.image_processing(image=cv_image)[1]
            
	    # establish minimum pixel area for a grape bunch
	    min_area = 50

	    # iterate through the list of contours
	    for i, contour in enumerate(contours):

	        # ignore the contour of the boundary of the image
	        if hierarchy[0][i][3] >= 0:

	            # include the contour if it is larger than the minimum area for a grape bunch
	            if cv2.contourArea(contour) >= min_area:

                        # update the variable found_grapes - useful for testing and debugging
	                self.found_grapes = True
	                
	                # get the coordinates of the grape centre in camera coordinates
	            	camera_coords = self.get_centroid(grape_contour=contour, image=cv_image, depth=cv_depth)
                        # skip the contour if the depth value is nan or the pixel index is out of range
	            	if camera_coords == None:
	            	    continue
	            	
                        # convert the grape centroid from camera coordinatesto world coordinates
                        grape_map_centroid = self.image_world_tf(time=image_time, coords=camera_coords) 
                                               
                        # visualise the grape centroid on rviz as a point cloud
                        self.point_cloud_projection(time=image_time, centroid=grape_map_centroid)
                                           
                        # round the grape centroid map coordinates to 1dp to improve readability during testing
                        grape_map_x = round(grape_map_centroid.point.x, 3)
                        grape_map_y = round(grape_map_centroid.point.y, 3)
                        grape_map_z = round(grape_map_centroid.point.z, 3)
                        grape_map_coords = [grape_map_x, grape_map_y, grape_map_z]

                        # check if the grape has already been counted and if not, increase count by 1
                        new_grape_detected = True

                        # if no grapes have been detected yet, save the grape coordinates to keep track of its location
                        if len(self.counted_grape_coords) == 0:
                            self.counted_grape_coords.append(grape_map_coords)
                            # print the rounded coordinates of the grape centroid
                            print 'Map coords: ', grape_map_coords
                            # increase the count by 1
                            self.count += 1
                            # set new_grape_detected to False so that the first grape is not counted twice
                            new_grape_detected = False

                        # if at least one grape has been counted before, check that the coordinates of the new grape are not too close to grapes already counted
                        else:
                            for counted_grape in self.counted_grape_coords:

                                # set a threshold for the closest distance to start counting the contour as a new grape
                                proximity_threshold = 0.08

                                # if the contour is within the threshold along the x- and z- axes, do not count it as a new grape
                                if (abs(grape_map_coords[0] - counted_grape[0]) < proximity_threshold) and (abs(grape_map_coords[2] - counted_grape[2]) < proximity_threshold):
                                    new_grape_detected = False

                        # if the grape is a new grape, save its coordinates and increase the count by 1
                        if new_grape_detected:
                            # save the grape coordinates to keep track of its location
                            self.counted_grape_coords.append(grape_map_coords)
                            # print the rounded coordinates of the grape centroid
                            print 'Map coords: ', grape_map_coords
                            # increase the count by 1
                            self.count += 1

            # print the number of grapes detected in this image analysis and the previous image analysis
            print 'Number of grape bunches: ', self.count
            print 'Previous count: ', self.previous_count

            # resume navigation once the count has been completed
            if self.previous_count == self.count:
                # publish feedback to the control node
                self.counting_status_pub.publish('counting complete')
            
        # publish the number of grape bunches to the control node and reset the grape count
        if self.reset_counting:
            self.reset_count()
            
rospy.init_node('grape_counter')
gc = grape_counter()
rospy.spin()		



