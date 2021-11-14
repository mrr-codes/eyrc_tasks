#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from aruco_library import *


class image_proc():

    # Initialise everything
    def __init__(self):
        rospy.init_node('marker_detection')  # Initialise rosnode

        # Making a publisher

        self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)

        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # Subscribing to /camera/camera/image_raw

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/camera/camera/image_raw", Image, self.image_callback)

        # -------------------------Add other ROS Subscribers here----------------------------------------------------

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()

        # This will contain the message structure of message type task_1/Marker
        self.marker_msg = Marker()

        self.rate = rospy.Rate(10)
    # Callback function of camera topic

    def calcuate_centre(self, corners):
        #np_arr = Detected_ArUco_markers[_id]
        np_arr = corners
        tl = np_arr[0, 0]
        #tr = np_arr[0, 1]
        br = np_arr[0, 2]
        #bl = np_arr[0, 3]

        ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return ctr

    def image_callback(self, data):
        # Note: Do not make this function lenghty, do all the processing outside this callback function
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # my code here
            Detected_ArUco_markers = detect_ArUco(self.img)
            angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
            # self.marker_msg.z = 0
            # self.marker_msg.roll = 0
            # self.marker_msg.pitch = 0

            for key in Detected_ArUco_markers.keys():
                centre = self.calcuate_centre(Detected_ArUco_markers[key])
                self.marker_msg.id = key
                self.marker_msg.x = centre[0]
                self.marker_msg.y = centre[1]
                self.marker_msg.yaw = angle[key]
                self.publish_data()

        except CvBridgeError as e:
            print(e)
            return 

    def publish_data(self):
        self.marker_pub.publish(self.marker_msg)
	rospy.loginfo(self.marker_msg)
	print(self.marker_msg)
        self.rate.sleep()


if __name__ == '__main__':
    image_proc_obj = image_proc()            # self.marker_msg.z = 0
    # self.marker_msg.roll = 0
    # self.marker_msg.pitch = 0
    rospy.spin()
