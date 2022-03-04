#!/usr/bin/env python3
"""
* Team Id : SS-1403
* Author List : Manas Reddy Ramidi, Juvvi Manas Sashank, Rikin Ramachandran, Soham Biswas
* Filename: SS_1234_strawberry_stacker.py
* Theme: Strawbeery stacker
* Classes and Functions: offboard_control,stateMoniter,image_processing,drone_0,drone_1
* Global Variables: NONE
"""

""" Task Implementation gist: We are splitting spawn_info data & accessing it in two separate lists. One list is assigned to drone0 & other
to drone1. After a split row number is passed to a function that inputs row no, the drone number returns a 
setpoint specific to the drone & on the frequency of box spawn. Lists are populated with tuples of row setpoints.
After drones take off to 3 m height, the row setpoint lists are added to the drone setpoints list.
After reaching the setpoint, velocity control kicks in, moving the drones straight along the row with a velocity
of 1.5 m/s. Along the way once a box is in the field of view the drone lowers to a height of 1 m and precise
positioning takes place by  PID algorithm for velocities that takes into account distance between the camera centre 
& aruco centre and the drone's current height. When the centre is to the left of the circle(the distance between 
the aruco centre and bottom corner is the radius of the circle), the autoland is activated and the gripper is 
activated on the box. During the land process the id of the current box is sent to a function that calculates truck 
setpoints based on the drone number, aruco id, and we append the points to the setpoint list of the drone. Once the
drone reaches near the truck gridpoint it goes to a lower height, drops the box, setpoint for drone to raise
to a height is appended.Next row start setpoints are calculated and the process repeats."""

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import Gripper

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import math
from multiprocessing import Process


class offboard_control:

    def __init__(self):
        rospy.init_node('offboard_control', anonymous=True)

    '''* Function Name: setArm_0
    * Output:Arms the drone
    * Logic: arming service is called
    * Example Call: setArm_0()'''
    def setArm_0(self):
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')

        print('setArm_0 called')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService_0 = rospy.ServiceProxy(
                '/edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_0(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)
        # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
    def setArm_1(self):
        rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        print('setArm_1 called')
        try:
            armService_1 = rospy.ServiceProxy(
                '/edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_1(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)
    '''* Function Name: offboard_set_mode_0
    * Output:Drone mode is changed to offboard
    * Logic: offboard service is called
    * Example Call: offboard_set_mode_0'''
    def offboard_set_mode_0(self):
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:

            set_ModeService_0 = rospy.ServiceProxy(
                'edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
            set_ModeService_0(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print("Service setting mode call failed: %s" % e)

    def offboard_set_mode_1(self):

        rospy.wait_for_service('/edrone1/mavros/set_mode')
        try:

            set_ModeService_1 = rospy.ServiceProxy(
                'edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            set_ModeService_1(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print("Service setting mode call failed: %s" % e)
    '''* Function Name:setAutoLandMode_0 
    * Output:Drone mode is changed to autoland
    * Logic: Autoland service is called
    * Example Call: setAutoLandMode_0()'''
    def setAutoLandMode_0(self):
        rospy.wait_for_service('/edrone0/mavros/set_mode')

        set_ModeService_0 = rospy.ServiceProxy(
            '/edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService_0(custom_mode='AUTO.LAND')
        print("d0 inside autoland")


    def setAutoLandMode_1(self):
        rospy.wait_for_service('/edrone1/mavros/set_mode')

        set_ModeService_1 = rospy.ServiceProxy(
            '/edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService_1(custom_mode='AUTO.LAND')
        print("d1 inside autoland")
    '''* Function Name: gripper_activate_0
    * Output:Drone gripper is activated or de-activated
    * Logic: gripper service is called
    * Example Call: gripper_activate_0(True)'''
    def gripper_activate_0(self, grip_control):
        rospy.wait_for_service('/edrone0/activate_gripper')
        gripper = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function_0")

    def gripper_activate_1(self, grip_control):
        rospy.wait_for_service('/edrone1/activate_gripper')
        gripper = rospy.ServiceProxy('/edrone1/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function_1")


class stateMoniter:
    def __init__(self):
        self.state_0 = State()
        self.pos_0 = PositionTarget()
        self.local_pos_0 = Point(0, 0, 0)

        self.state_1 = State()
        self.pos_1 = PositionTarget()
        self.local_pos_1 = Point(0, 0, 0)

        self.row_spawn_sp0 = list() #list of row start coordinates for drone0 
        self.row_spawn_sp1 = list() #list of row start coordinates for drone1
        self.spawn_count = 0 # counter for box rows
       
        self.bt_0 = -1 #index variable of blue truck cells for drone0
        self.rt_0 = -1 #index variable of red truck cells for drone0
        self.bt_1 = 0 #index variable of blue truck cells for drone1
        self.rt_1 = 0 #index variable of red truck cells for drone1
        self.box_counts = dict()

        #The grid cell setpoint values of the blue truck 
        self.blue_truck = np.array([[(13.85, -7.4, 1.84), (13.85, -6.17, 1.84), (13.85, -4.95, 1.84)], [(14.7, -7.4, 1.84), (14.7, -6.17, 1.84),
                                   (14.7, -4.95, 1.84)], [(15.55, -7.4, 1.84), (15.55, -6.17, 1.84), (15.55, -4.95, 1.84)], [(16.4, -7.4, 1.84), (16.4, -6.17, 1.84), (16.4, -4.95, 1.84)]])

        #The grid cell setpoint values of the red truck
        self.red_truck = np.array([[(56.5, 64.75, 1.84), (56.5, 65.98, 1.84), (56.5, 67.21, 1.84)], [(57.35, 64.75, 1.84), (57.35, 65.98, 1.84), (57.35, 67.21, 1.84)],
                                  [(58.2, 64.75, 1.84), (58.2, 65.98, 1.84), (58.2, 67.21, 1.84)], [(59.05, 64.75, 1.84), (59.05, 65.98, 1.84), (59.05, 67.21, 1.84)]])

        
        #Sequencing of dropping the boxes for each truck
        self.blue_truck_seq = [self.blue_truck[3, 2], self.blue_truck[3, 1], self.blue_truck[3, 0], self.blue_truck[2, 0], self.blue_truck[2, 1], self.blue_truck[
            2, 2], self.blue_truck[1, 2], self.blue_truck[1, 1], self.blue_truck[1, 0]]

        self.red_truck_seq = [self.red_truck[3, 2], self.red_truck[3, 1], self.red_truck[3, 0], self.red_truck[2, 0], self.red_truck[2, 1], self.red_truck[
            2, 2], self.red_truck[1, 2], self.red_truck[1, 1], self.red_truck[1, 0]]


    #Callback for state and pose and gripper_check for both drones
    def stateCb_0(self, msg):
        self.state_0 = msg

    def stateCb_1(self, msg):
        self.state_1 = msg

    def posCb_0(self, msg):
        self.local_pos_0.x = msg.pose.position.x
        self.local_pos_0.y = msg.pose.position.y
        self.local_pos_0.z = msg.pose.position.z

    def posCb_1(self, msg):
        self.local_pos_1.x = msg.pose.position.x
        self.local_pos_1.y = msg.pose.position.y
        self.local_pos_1.z = msg.pose.position.z

    def gripper_check_clbk_0(self, msg):
        self.check_gripper_0 = msg.data

    def gripper_check_clbk_1(self, msg):
        self.check_gripper_1 = msg.data
        
# Function for calculating row search start setpoints from row no. and drone no and the number of boxes in a particular row.
    '''* Function Name:calculate_row_start
    * Input:row_no,drone_no
    * Output:Calculates row start point
    * Logic: row_no and drone_no are passed as inputs and row start points are calculated based on number of boxes in that row
    * Example Call: calculate_row_start(msg.data,1)'''
    def calculate_row_start(self, row_no, drone_no):
        
        self.boxes_in_row = self.box_counts[row_no]
        if drone_no == 0:
            if self.boxes_in_row> 4:
                return (38, 4*(row_no-1), 3)
            elif self.boxes_in_row > 3:
                return (30, 4*(row_no-1), 3)
            elif self.boxes_in_row > 2:
                return (19, 4*(row_no-1), 3)
            elif self.boxes_in_row > 1:
                return (9, 4*(row_no-1), 3)
            else:
                return(-1,4*(row_no-1),3)
        else:
            if self.boxes_in_row>4:
                return (38, 4*(row_no-16), 3)
            elif self.boxes_in_row > 3:
                return (30, 4*(row_no-16), 3)
            elif self.boxes_in_row >2:
                return (19, 4*(row_no-16), 3)
            elif self.boxes_in_row > 1:
                return (9, 4*(row_no-16), 3)    
            else:
                return(-2,4*(row_no-16),3)
#Callback of spawn info topic
    '''* Function Name: spawn_clbk
    * Input: msg (spawn_info data)
    * Output: Appends the row start points to a list
    * Logic:Spawn_info data is split alternatively between the drones and row start values are appended to a list
    * Example Call: Callback function'''
    def spawn_clbk(self, msg):
        #Dictionary to keep a track of number of boxes in a particular row
        self.box_counts[msg.data] = self.box_counts.get(msg.data, 0) + 1
        #Splitting the spawn_info data into lists,calculating the row start and appending the values to a list
        if self.spawn_count % 2 == 0:
            self.row_spawn_sp0.append(self.calculate_row_start(msg.data, 0))
            print('d0 Row_spawn list',self.row_spawn_sp0)

        else:
            self.row_spawn_sp1.append(self.calculate_row_start(msg.data, 1))
            print('d1 Row_spawn list',self.row_spawn_sp1)
        #Incrementing the spawn_count after calculating row start values    
        self.spawn_count += 1
        
    '''* Function Name: calculate_truck_point
    * Input: id-> id of box, drone_no.
    * Output: Array of truck grid setpoint tuples
    * Logic: calculates desired truck points based on inputs
    * Example Call: example = calculate_truck_point(1,1)'''
    def calculate_truck_point(self, id, drone_no):
        if id == 2:  # blue box
            if drone_no == 0:
                self.bt_0 += 1 # Accesing the blue truck array values from the top  for drone 0 to avoid crashing with drone1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.blue_truck_seq[self.bt_0], (-1, 1, 0)))#For drone0, if box is blue(id=2),calculating box drop setpoint wrt. drone0 initial point and blue truck grid point global coordinates
                final_array = [(drop_pt[0], drop_pt[1],5),
                                (drop_pt[0], drop_pt[1],5)]#extracting the x,y coordinates of blue truck and creating a list which has two points for the drone to reach near the truck,and after ungripping the to raise to a height again

            else:
                self.bt_1 -= 1 # Accesing the blue truck array values from the bottom for drone 1 to avoid crashing with drone0
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.blue_truck_seq[self.bt_1], (-1, 61, 0)))#For drone1, if box is blue(id=2),calculating box drop setpoint wrt. drone0 initial point and blue truck grid point global coordinates

                final_array = [(drop_pt[0], drop_pt[1], 6),
                                (drop_pt[0], drop_pt[1], 6)]#extracting the x,y coordinates of blue truck and creating a list which has two points for the drone to reach near the truck,and after ungripping the to raise to a height again

        else:
            if drone_no == 0:
                self.rt_0 += 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.red_truck_seq[self.rt_0], (-1, 1, 0))) #For drone0, if box is red(id=1),calculating box drop setpoint wrt. drone0 initial point and red truck grid point global coordinates
                final_array = [(drop_pt[0], drop_pt[1], 5), (drop_pt[0], drop_pt[1], 5)]


            else:
                self.rt_1 -= 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.red_truck_seq[self.rt_1], (-1, 61, 0))) #For drone1, if box is red(id=1),calculating box drop setpoint wrt. drone1 initial point and red truck grid point global coordinates
                final_array = [(drop_pt[0], drop_pt[1], 6),
                               (drop_pt[0], drop_pt[1], 6)]

        return final_array


class image_processing:

    def __init__(self):

        self.img = np.empty([])
        self.bridge = CvBridge()
        self.position_aruco_x_0 = 1000 
        self.position_aruco_y_0 = 1000
        self.position_aruco_x_1 = 1000
        self.position_aruco_y_1 = 1000
        self.aruco_thresh_bool = False
        self.box_setpoint = list()
        self.Detected_ArUco_markers_0 = []
        self.Detected_ArUco_markers_1 = []
        self.bcorner_0 = []
        self.bcorner_1 = []
        self.exo_rad_0 = 0
        self.exo_rad_1 = 0
        
    '''* Function Name: detect_Aruco
    * Input: img->cv2 img data
    * Output: dictionary of id and corners
    * Logic: apply cv2, aruco libraries to extract data 
    * Example Call: example = detect_Aruco(img)''' 
    def detect_ArUco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #converting the image to a grayscale image
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)#aruco detection function is called, returns the ID and Corners

        Detected_ArUco_markers_func = {}
        if ids != None:

            Detected_ArUco_markers_func = dict(zip(ids[:, 0], corners))

        return Detected_ArUco_markers_func
    '''* Function Name: calculate_centre
    * Input: corners(list of corners of aruco)
    * Output:Centre of the aruco, orientation of aruco in degrees
    * Logic: The centre is calculated based on corner values,and the orientation is calculated based on differences between corner values using atan function
    * Example Call: calcuate_centre(self.Detected_ArUco_markers_0[key])'''
    #A function to calculate the centre of the AruCo Marker
    def calcuate_centre(self, corners):
        np_arr = corners
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]
        bl = np_arr[0, 3]
        self.bcorner_0 = br
        self.bcorner_1 = br
        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        diffs = [(tl[0]-bl[0]), (tl[1]-bl[1])]
        degree = math.atan2(diffs[0], diffs[1])*180/math.pi
        final_degree = degree+float(270)
        if (final_degree > 360.0):
            final_degree -= 360.0
        ArUco_marker_angles = final_degree 
        return self.ctr, ArUco_marker_angles
    '''* Function Name:image_callback_0
    * Input: data(an image frame from the drone camera)
    * Output: The image frame with the centre marked and an exocircle on the aruco marker and also displays the angle of the Aruco
    * Logic: The image is converted to CV2 format and the centre is calculated using corner points
    *        After this an exocircle is drawn with the CV2 circle function with centre of Aruco as the centre of the circle
    * Example Call: Callback Function'''
    #Callback function for drone 0 to convert the image to CV2 format and drawing an exocircle on the frame
    def image_callback_0(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)
           
            self.Detected_ArUco_markers_0 = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers_0.keys():
                self.centre, angle_detected = self.calcuate_centre(
                    self.Detected_ArUco_markers_0[key])

                self.position_aruco_x_0 = self.centre[0] #x-coordinate of centre of aruco in pixels
                self.position_aruco_y_0 = self.centre[1] #y-coordinate of centre of aruco in pixels
                self.bcorner_0 = self.bcorner_0 #(x,y) coordinates of bottom corner of aruco in pixels
                self.exo_rad_0=math.sqrt((self.bcorner_0[0]-self.position_aruco_x_0)**2+(self.bcorner_0[1]-self.position_aruco_y_0)**2)# calculating radius of exo-circle(euclidean diatance between centre and bottom corner)
                cv2.circle(self.img,(int(self.position_aruco_x_0),int(self.position_aruco_y_0)),radius=abs(int(self.exo_rad_0)),color= (0,225,0),thickness=2)#drawing an exo circle around the box with exo_rad_1 as the radius 

                samp = str(round(angle_detected))
                cv2.putText(self.img, samp, ((int(self.ctr[0])-90), int(self.ctr[1])),fontFace=cv2.FONT_HERSHEY_COMPLEX, thickness=2, fontScale=0.75, color=(0, 255, 0))

            cv2.imshow('check_frame_0', self.img)#drone_0 camera feed
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return
    #Callback function for drone 1 to convert the image to CV2 form an drawing an exocircle on the frame
    '''* Function Name:image_callback_1
    * Input: data(an image frame from the drone camera)
    * Output: The image frame with the centre marked and an exocircle on the aruco marker and also displays the angle of the Aruco
    * Logic: The image is converted to CV2 format and the centre is calculated using corner points
    *        After this an exocircle is drawn with the CV2 circle function with centre of Aruco as the centre of the circle
    * Example Call: Callback Function'''
    def image_callback_1(self, data):   

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)

            self.Detected_ArUco_markers_1 = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers_1.keys():
                self.centre, angle_detected = self.calcuate_centre(
                    self.Detected_ArUco_markers_1[key])
                self.position_aruco_x_1 = self.centre[0] #x-coordinate of centre of aruco in pixels
                self.position_aruco_y_1 = self.centre[1] #y-coordinate of centre of aruco in pixels
                self.bcorner_1 = self.bcorner_1#(x,y) coordinates of bootom corner of aruco in pixels
                self.exo_rad_1=math.sqrt((self.bcorner_1[0]-self.position_aruco_x_1)**2+(self.bcorner_1[1]-self.position_aruco_y_1)**2)# calculating radius of exo-circle(euclidean diatance between centre and bottom corner)
                cv2.circle(self.img,(int(self.position_aruco_x_1),int(self.position_aruco_y_1)),radius=abs(int(self.exo_rad_1)),color= (0,225,0),thickness=2)#drawing an exo circle around the box with exo_rad_1 as the radius          
                

                samp = str(round(angle_detected))
                cv2.putText(self.img, samp, ((int(self.ctr[0])-90), int(self.ctr[1])),fontFace=cv2.FONT_HERSHEY_COMPLEX, thickness=2, fontScale=0.75, color=(0, 255, 0))
                
            cv2.imshow('check_frame_1', self.img)#drone_1 camera feed
            cv2.waitKey(1)               
            

        except CvBridgeError as e:
            print(e)
            return

'''* Function Name: drone_0
* Input: None
* Output: The operation of Drone 0
* Logic:After drones take off to 3 m height, the row setpoint lists are added to the drone setpoints list.
        After reaching the setpoint, velocity control kicks in, moving the drones straight along the row with a velocity
        of 1.5 m/s. Along the way once a box is in the field of view the drone lowers to a height of 1 m and precise
        positioning takes place by  PID algorithm for velocities that takes into account distance between the camera centre 
        & aruco centre and the drone's current height. When the centre is to the left of the circle(the distance between 
        the aruco centre and bottom corner is the radius of the circle), the autoland is activated and the gripper is 
        activated on the box. During the land process the id of the current box is sent to a function that calculates truck 
        setpoints based on the drone number, aruco id, and we append the points to the setpoint list of the drone. Once the
        drone reaches near the truck gridpoint it goes to a lower height, drops the box, setpoint for drone to raise
        to a height is appended.Next row start setpoints are calculated and the process repeats. 
'''
#function for operating Drone 0
def drone_0():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()

    # Initialize publishers
    local_pos_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # Intializing Subscribers 
    rospy.Subscriber("/edrone0/mavros/state", State, stateMt.stateCb_0)
    rospy.Subscriber("/edrone0/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_0)
    rospy.Subscriber('/edrone0/gripper_check', String,
                     stateMt.gripper_check_clbk_0)
    rospy.Subscriber("/edrone0/camera/image_raw",
                     Image, img_proc.image_callback_0)
    rospy.Subscriber('/spawn_info', UInt8, stateMt.spawn_clbk)
    rate = rospy.Rate(20.0)

    setpoints_0 = [(0, 0, 3)]#setpoints list
    # Create empty message containers for position and velocity messages 
    pos_0 = PoseStamped()
    pos_0.pose.position.x = 0
    pos_0.pose.position.y = 0
    pos_0.pose.position.z = 0

    vel_0 = TwistStamped()
    vel_0.twist.linear.x = 0
    vel_0.twist.linear.y = 0
    vel_0.twist.linear.z = 0
    
    # publishing dummy points to the drone
    def dummy_points_0():
        for i in range(100):
            print('d0 Sending dummy points')
            local_pos_pub_0.publish(pos_0)
            rate.sleep()
    dummy_points_0()
    
    # function to arm the drone
    def arm_0():
        while not stateMt.state_0.armed:
            ofb_ctl.setArm_0()
            rate.sleep()
        print("d0 Armed!!")
        ofb_ctl.setArm_0()
    arm_0()

    # Enabling the OFFBOARD mode
    def offboard_0():
        while not stateMt.state_0.mode == "OFFBOARD":
            ofb_ctl.offboard_set_mode_0()
            rate.sleep()
        print("d0 OFFBOARD mode activated")
    offboard_0()

    i = 0 #Variable to iterate through the setpoints list
    pos_0.pose.position.x = setpoints_0[i][0]
    pos_0.pose.position.y = setpoints_0[i][1]
    pos_0.pose.position.z = setpoints_0[i][2]
    '''* Function Name: check_position_0
    * Input: None
    * Output: Boolean
    * Logic: Continuously subtracts the current position of drone from the current setpoint 
      returns true on reaching threshold    
    * Example Call: example_position= check_position_0'''
    # Checking the accuracy of the drone position to desired position
    def check_position_0():
        desired = np.array(
            (setpoints_0[i][0], setpoints_0[i][1], setpoints_0[i][2]))
        pos = np.array((stateMt.local_pos_0.x,
                        stateMt.local_pos_0.y,
                        stateMt.local_pos_0.z))
        #print('d0', np.linalg.norm(desired - pos))
        if (i > 3 and i == (len(setpoints_0) - 3)) or (i==1):
            return np.linalg.norm(desired - pos) < 0.2
        else:
            return np.linalg.norm(desired - pos) < 0.5
    
    land_count = 0 # No. of times drone landed
    flag1 = False #Boolean to start velocity control after drone reaches start of the row,if true row start velocity is published
    previous_y_error = 0 #Variable to keep track of the error in y-direction of the previous timestep
    vi = 0 #Integral gain term
    box_dropped = True #Boolean to ensure that boxes are picked up only after drone reaches row_start points and velocity is published,if false boxes are picked up
    flag_flip_pos_vol = False #Boolean to track velocity control once the drone reaches row start
    k = 0 #Variable to iterate through the row_spawn list
    m = -1 #Variable to make the drone lower height after box is in required limits, only once
    x = 0 #Variable condition for adding the first row search setpoint
    ofb_ctl.setArm_0()
    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.offboard_set_mode_0()
        reached = check_position_0()

        # Condition for clearing the previous setpoints list then append row search start setpoints, once a box is dropped 
        if i > 4 and (len(setpoints_0)-1):
            print('d0 clearing spts.')
            setpoints_0.clear()
            i = 0
            k += 1
            previous_y_error = 0
            try:
                setpoints_0.extend([(stateMt.local_pos_0.x,stateMt.local_pos_0.y,6),stateMt.row_spawn_sp0[k],(0,0,4)])#Raising drone to a height of 6m setpoint,appending Kth element of row_spawn list and (0,0,4) works as a dummy setpoint for check_position func
            except:
                print("Row_spawn_0 is empty")
            print('d0 Setpoints list as of now', setpoints_0)

        # Setting necessary variables on reaching row search start
        if  i==2 and flag_flip_pos_vol == True :
            box_dropped = False
            m = -1
            print("d0 setting m to -1")

        #if the aruco marker is detected and there is no box gripped
        if len(img_proc.Detected_ArUco_markers_0) > 0 and box_dropped == False:
            flag_flip_pos_vol = False
            img_proc.aruco_thresh_bool = True # If Aruco detected: True
            
            if (m < 0):
                #If drone is within close vicinity to box centre, lowering of height will be executed once
                if 150 < img_proc.position_aruco_x_0 < 250:

                    print('d0 publishing set pt to decrease height to 1m')
                    pos_0.pose.position.x = stateMt.local_pos_0.x
                    pos_0.pose.position.y = stateMt.row_spawn_sp0[k][1]
                    pos_0.pose.position.z = 1.5
                    local_pos_pub_0.publish(pos_0)
                    rospy.sleep(3)
                    m += 1
            #After lowering,PID velocitiy cofntrol for precise centring will take place based on distance from Aruco centre to camera centre and the current height of the drone 
            if m == 0 :
                if (225-img_proc.position_aruco_y_0)<0:
                    vi = 0.2
                else:
                    vi = 0
                vel_0.twist.linear.x = (
                    ((img_proc.position_aruco_x_0 - 200)*stateMt.local_pos_0.z)/300)
                vel_0.twist.linear.y = -((((img_proc.position_aruco_y_0 - (200 + 80/stateMt.local_pos_0.z))*stateMt.local_pos_0.z)/400) - (
                    img_proc.position_aruco_y_0 - (200 + 80/stateMt.local_pos_0.z) - previous_y_error)/40)-vi
                vel_0.twist.linear.z = (1.5-stateMt.local_pos_0.z)/20
                local_vel_pub_0.publish(vel_0)

            # When camera centre aligns under desired area inside the exocircle, box gripping sequence starts
            if(((200 - img_proc.position_aruco_x_0)**2 + (225-img_proc.position_aruco_y_0)**2)<= (img_proc.exo_rad_0)**2) and (225 <img_proc.position_aruco_y_0):
                flag1 = True
                box_id = list(img_proc.Detected_ArUco_markers_0.keys())[0]

                img_proc.box_setpoint = [
                    stateMt.local_pos_0.x, stateMt.local_pos_0.y] #Position of the aruco in the field
                print('d0 Box is at ', img_proc.box_setpoint)
                #lowering to a height of 1m where the aruco is placed
                pos_0.pose.position.x = img_proc.box_setpoint[0]
                pos_0.pose.position.y = img_proc.box_setpoint[1]
                pos_0.pose.position.z = 1
                local_pos_pub_0.publish(pos_0)
                ofb_ctl.setAutoLandMode_0()
                print('d0 Attempted to land c=', str(land_count))
                #loop to ensure proper gripping 
                while not stateMt.check_gripper_0 == 'True':
                    if stateMt.local_pos_0.z < 0.25:
                        ofb_ctl.gripper_activate_0(True)
                stateMt.boxes_in_row -= 1  # deducting 1 from dictionary after picking a box
                if stateMt.check_gripper_0 == 'True':
                    print('d0 The box has been gripped')
                    land_count += 1
                    box_dropped = True
                else:
                    print('d0 The box cannot yet be gripped')

                img_proc.aruco_thresh_bool = False
                offboard_0()
                setpoint = (stateMt.local_pos_0.x,
                            stateMt.local_pos_0.y, 3)
                setpoints_0.insert(i+1, setpoint) #Inserting setpoint for drone to raise to a height after picking up the box
                i = i+1
                print('d0 i increased to ', i, 'after re-arming')
                print('d0 Setting flag1 to false again')
                flag1 = False
                local_pos_pub_0.publish(pos_0)
                
                truck_pts = stateMt.calculate_truck_point(box_id, 0)#Calculating truck_setpoint based on the id of the box and drone number
                setpoints_0.extend(
                    [truck_pts[0], truck_pts[1], (0, 0, 0)])#Appending truck_setpoints and a dummy point for check_position func
                print('d0 Setpoints list as of now', setpoints_0)

            previous_y_error = img_proc.position_aruco_y_0 - \
                (200 + 80/stateMt.local_pos_0.z)#Calculating the previous y error based on distance between aruco and drone,height

        elif img_proc.aruco_thresh_bool == False:
            ofb_ctl.offboard_set_mode_0()

            pos_0.pose.position.x = setpoints_0[i][0]
            pos_0.pose.position.y = setpoints_0[i][1]
            pos_0.pose.position.z = setpoints_0[i][2]
            
            #Condition for checking whether drone is at row start i.e coordinates in multiples of 4, and then executing velocity control
            if reached == True and  x!= 0 and (abs(setpoints_0[i][1]) % 4 == 0):
                vel_0.twist.linear.x = 1.5
                vel_0.twist.linear.y = 0
                vel_0.twist.linear.z = -0.15
                
                flag_flip_pos_vol = True  # have to turn the boolean to false if aruco detected
            #Publishing row start velocity
            if flag_flip_pos_vol == True:
                local_vel_pub_0.publish(vel_0)
                box_dropped = False

            else:
                local_pos_pub_0.publish(pos_0)

            # Condition if drone has reached given setpoint and then to increment setpoints iterator
            if reached == True and flag1 == False:
                print("d0 Reached goal")
                #For adding initial points once
                while x == 0:
                    setpoints_0.extend([stateMt.row_spawn_sp0[k],(0,4,4)])
                    print('d0 after reaching goal setpoints are', setpoints_0)
                    x = x+1
                    #rospy.sleep(2)

                i = i+1
                print('d0 i increased to ', i, 'after reaching goal')
            # Condition whether the drone has reached truck drop point and then to land to drop the box
            if i > 3 and i == (len(setpoints_0) - 2):
                ofb_ctl.setAutoLandMode_0() 
                #Loop to ensure that ungripping doesn't happen when drone height is > 2.3m
                while not stateMt.local_pos_0.z < 2.3:
                    loop=0
                for o in range(3):
                    ofb_ctl.gripper_activate_0(False)
                    
                box_dropped = True #This will ensure that boxes on the truck are not picked up 
                print("d0 Releasing box")

        rate.sleep()
'''* Function Name: drone_1
* Input: None
* Output: The operation of Drone 1
* Logic:After drones take off to 3 m height, the row setpoint lists are added to the drone setpoints list.
        After reaching the setpoint, velocity control kicks in, moving the drones straight along the row with a velocity
        of 1.5 m/s. Along the way once a box is in the field of view the drone lowers to a height of 1 m and precise
        positioning takes place by  PID algorithm for velocities that takes into account distance between the camera centre 
        & aruco centre and the drone's current height. When the centre is to the left of the circle(the distance between 
        the aruco centre and bottom corner is the radius of the circle), the autoland is activated and the gripper is 
        activated on the box. During the land process the id of the current box is sent to a function that calculates truck 
        setpoints based on the drone number, aruco id, and we append the points to the setpoint list of the drone. Once the
        drone reaches near the truck gridpoint it goes to a lower height, drops the box, setpoint for drone to raise
        to a height is appended.Next row start setpoints are calculated and the process repeats. 
'''
# Function for Drone 1
def drone_1():
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()
    # Initialize publishers
    local_pos_pub_1 = rospy.Publisher(
        '/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_1 = rospy.Publisher(
        '/edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Intializing Subscribers 
    rospy.Subscriber("/edrone1/mavros/state", State, stateMt.stateCb_1)
    rospy.Subscriber("/edrone1/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_1)
    rospy.Subscriber('/edrone1/gripper_check', String,
                     stateMt.gripper_check_clbk_1)
    rospy.Subscriber("/edrone1/camera/image_raw",
                     Image, img_proc.image_callback_1)
    rospy.Subscriber('/spawn_info', UInt8, stateMt.spawn_clbk)

    rate = rospy.Rate(20.0)

    setpoints_1= [(0, 0, 3)]#setpoints list
    # Create empty message containers for position and velocity messages 
    pos_1 = PoseStamped()
    pos_1.pose.position.x = 0
    pos_1.pose.position.y = 0
    pos_1.pose.position.z = 0

    vel_1 = TwistStamped()
    vel_1.twist.linear.x = 0
    vel_1.twist.linear.y = 0
    vel_1.twist.linear.z = 0

    # Publishing Dummy points to the drone
    def dummy_points_1():
        for i in range(100):
            print('d1 Sending dummy points')
            local_pos_pub_1.publish(pos_1)
            rate.sleep()
    dummy_points_1()
    
    # Arming the drone
    def arm_1():
        while not stateMt.state_1.armed:
            ofb_ctl.setArm_1()
            rate.sleep()
        ofb_ctl.setArm_1()
        print("d1 Armed!!")
    arm_1()

    # Setting the drone to OFFBOARD mode
    def offboard_1():
        while not stateMt.state_1.mode == "OFFBOARD":
            ofb_ctl.offboard_set_mode_1()
            rate.sleep()
        print("d1 OFFBOARD mode activated")
    offboard_1()

    i = 0#Variable to iterate through the setpoints list
    pos_1.pose.position.x = setpoints_1[i][0]
    pos_1.pose.position.y = setpoints_1[i][1]
    pos_1.pose.position.z = setpoints_1[i][2]
    '''* Function Name: crash_func
    * Input: None
    * Output: Rospy.sleep for 2 seconds for drone_1
    * Logic: Continuously subtracts the current position of drone_0 from drone_1
      drone_1 stops if it is too close to drone_0    
    * Example Call: crash_func()'''
    def crash_func():
        drone_0 = np.array((stateMt.local_pos_0.x,
                        stateMt.local_pos_0.y,
                        stateMt.local_pos_0.z))
        drone_1 = np.array((stateMt.local_pos_1.x,
                        stateMt.local_pos_1.y-60,
                        stateMt.local_pos_1.z))
        while np.linalg.norm(drone_0 - drone_1) < 0.5:
            rospy.sleep(1)
    '''* Function Name: check_position_1
    * Input: None
    * Output: Boolean
    * Logic: Continuously subtracts the current position of drone from the current setpoint 
      returns true on reaching threshold    
    * Example Call: example_position= check_position_1'''
    # Checking the accuracy of the drone position to desired position
    def check_position_1():
        desired = np.array(
            (setpoints_1[i][0], setpoints_1[i][1], setpoints_1[i][2]))
        pos = np.array((stateMt.local_pos_1.x,
                        stateMt.local_pos_1.y,
                        stateMt.local_pos_1.z))
        #print('d1', np.linalg.norm(desired - pos))
        if (i > 3 and i == (len(setpoints_1) - 3)) or (i==1):
            return np.linalg.norm(desired - pos) < 0.2
        else:
            return np.linalg.norm(desired - pos) < 0.5

    
    land_count = 0# No. of times drone landed 
    flag1 = False #Boolean to start velocity control after drone reaches start of the row,if true row start velocity is published
    previous_y_error = 0#Variable to keep track of the error in y-direction of the previous timestep
    vi = 0 #Integral gain term
    box_dropped = True #Boolean to ensure that boxes are picked up only after drone reaches row_start points and velocity is published,if false boxes are picked up
    flag_flip_pos_vol = False #Boolean to track velocity control once the drone reaches row start
    k = 0#Variable to iterate through the row_spawn list
    m = -1 #Variable to make the drone lower height after box is in required limits, only once
    x = 0 #Variable condition for adding the first row search setpoint
    ofb_ctl.setArm_1()

    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.offboard_set_mode_1()
        reached = check_position_1()
        crash_func()

        # Condition for clearing the previous setpoints list then append row search start setpoints, once a box is dropped
        if i > 4 and (len(setpoints_1)-1):
            print('d1 clearing spts.')
            setpoints_1.clear()
            i = 0
            k += 1
            previous_y_error = 0
            try:
                setpoints_1.extend([(stateMt.local_pos_1.x,stateMt.local_pos_1.y,7),stateMt.row_spawn_sp1[k],(0,0,4)])
            except:
                print("Row_spawn is empty")
            print('d1 Setpoints list as of now', setpoints_1)

        # Setting necessary variables on reaching row search start
        if  i==2 and flag_flip_pos_vol == True :
            print(i)
            box_dropped = False
            m = -1
            print("d1 setting m to -1")
            
        #if the aruco marker is detected and there is no box gripped
        if len(img_proc.Detected_ArUco_markers_1) > 0 and box_dropped == False:
            
            flag_flip_pos_vol = False
            img_proc.aruco_thresh_bool = True

            if (m < 0):
                flag_flip_pos_vol = False
                #If drone is within close vicinity to box centre, lowering of height will be executed once
                if 150 < img_proc.position_aruco_x_1 < 250:
                    print('d1 publishing set pt to decrease height to 1m')
                    pos_1.pose.position.x = stateMt.local_pos_1.x
                    pos_1.pose.position.y = stateMt.row_spawn_sp1[k][1]
                    pos_1.pose.position.z = 1.5
                    local_pos_pub_1.publish(pos_1)
                    rospy.sleep(3)
                    m += 1
            #After lowering,PID velocitiy cofntrol for precise centring will take place based on distance from Aruco centre to camera centre and the current height of the drone 
            if m==0:
                if (225-img_proc.position_aruco_y_1)<0:
                    vi = 0.2
                else:
                    vi = 0
                vel_1.twist.linear.x = (
                    ((img_proc.position_aruco_x_1 - 200)*stateMt.local_pos_1.z)/300)
                vel_1.twist.linear.y = -((((img_proc.position_aruco_y_1 - (200 + 80/stateMt.local_pos_1.z))*stateMt.local_pos_1.z)/400) - (
                    img_proc.position_aruco_y_1 - (200 + 80/stateMt.local_pos_1.z) - previous_y_error)/40)-vi
                vel_1.twist.linear.z = (1.5-stateMt.local_pos_1.z)/20
                print(vel_1.twist.linear.y,"drone1111111111111111")
                
                local_vel_pub_1.publish(vel_1)
                

    
            # When camera centre aligns under desired area inside the exocircle, box gripping sequence starts
            if(((200 - img_proc.position_aruco_x_1)**2 + (225-img_proc.position_aruco_y_1)**2)<= (img_proc.exo_rad_1)**2) and (225 <img_proc.position_aruco_y_1):
                flag1 = True
                box_id = list(img_proc.Detected_ArUco_markers_1.keys())[0]

                img_proc.box_setpoint = [
                    stateMt.local_pos_1.x, stateMt.local_pos_1.y]
                print('d1 Box is at ', img_proc.box_setpoint)
                #lowering to a height of 1m where the aruco is placed
                pos_1.pose.position.x = img_proc.box_setpoint[0]
                pos_1.pose.position.y = img_proc.box_setpoint[1]
                pos_1.pose.position.z = 1
                local_pos_pub_1.publish(pos_1)
                ofb_ctl.setAutoLandMode_1()
                print('d1 Attempted to land c=', str(land_count))
                while not stateMt.check_gripper_1 == 'True':
                     if stateMt.local_pos_1.z < 0.25:
                        ofb_ctl.gripper_activate_1(True)
                stateMt.boxes_in_row -= 1    
                if stateMt.check_gripper_1 == 'True':
                    print('d1 The box has been gripped')
                    land_count += 1
                    box_dropped = True
                else:
                    print('d1 The box cannot yet be gripped')

                img_proc.aruco_thresh_bool = False                
                offboard_1()
                setpoint = (stateMt.local_pos_1.x,
                            stateMt.local_pos_1.y, 4)
                setpoints_1.insert(i+1, setpoint) #Inserting setpoint for drone to raise to a height after picking up the box
                i = i+1
                print('d1 i increased to ', i, 'after re-arming')
                print('d1 Setting flag1 to false again')
                flag1 = False
                local_pos_pub_1.publish(pos_1)

                truck_pts = stateMt.calculate_truck_point(box_id, 1)#Calculating truck_setpoint based on the id of the box and drone number
                setpoints_1.extend(
                    [truck_pts[0], truck_pts[1],(0, 0, 0)])#Appending truck_setpoints and a dummy point for check_position func
                print('d1 Setpoints list as of now', setpoints_1)

            previous_y_error = img_proc.position_aruco_y_1 - \
                (200 + 80/stateMt.local_pos_1.z)#Calculating the previous y error based on distance between aruco and drone,height

        elif img_proc.aruco_thresh_bool == False:
            ofb_ctl.offboard_set_mode_1()

            pos_1.pose.position.x = setpoints_1[i][0]
            pos_1.pose.position.y = setpoints_1[i][1]
            pos_1.pose.position.z = setpoints_1[i][2]

            #Condition for checking whether drone is at row start i.e coordinates in multiples of 4, and then executing velocity control
            if reached == True and (setpoints_1[i][1] != 0 and abs(setpoints_1[i][1]) % 4 == 0):
                vel_1.twist.linear.x = 1.5
                vel_1.twist.linear.y = 0
                vel_1.twist.linear.z = -0.15

                flag_flip_pos_vol = True  # have to turn it false in aruco detected

            if flag_flip_pos_vol == True:
                print('d1 publishing row start velocity')
                local_vel_pub_1.publish(vel_1)
                box_dropped = False

            else:
                local_pos_pub_1.publish(pos_1)
            # Condition if drone has reached given setpoint and then to increment setpoints iterator
            if reached == True and flag1 == False:
                print("d1 Reached goal")
                #For adding initial points once
                while x == 0:
                    print(k)
                    print(stateMt.row_spawn_sp1)
                    print(stateMt.row_spawn_sp1[k])
                    setpoints_1.extend([stateMt.row_spawn_sp1[k],(0,4,4)])
                    print('after reaching goal setpoints are', setpoints_1)
                    x = x+1

                i = i+1
                print('d1 i increased to ', i, 'after reaching goal')
            #Conditions whether truck has reached the truck setpoint and 
            if i > 3 and i == (len(setpoints_1) - 2):
                ofb_ctl.setAutoLandMode_1()
                #Loop to ensure that ungripping doesn't happen when drone height is > 2.3m
                while not stateMt.local_pos_1.z<2.3:
                    loop=0              
                for o in range(3):    
                    ofb_ctl.gripper_activate_1(False)
                    
                box_dropped = True #This will ensure that boxes on the truck are not picked up 
                print("d1 Releasing box")

        rate.sleep()


if __name__ == '__main__':

    try:
        p1 = Process(target=drone_0) #Assigning the drone 0 fucntion as the first process
        p1.start() #starting process 1
        p2 = Process(target=drone_1)#Assigning the drone 1 fucntion as the second process
        p2.start()#starting process 2

        p1.join()
        p2.join()
    except rospy.ROSInterruptException:
        pass
