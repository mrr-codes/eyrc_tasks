#!/usr/bin/env python3


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
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
        # Waiting untill the service starts

    def paramset():
        rospy.wait_for_service('/edrone0/mavros/param/set')
        # rospy.wait_for_service('/edrone1/mavros/param/set')
        try:
            change_mode_0 = rospy.ServiceProxy(
                '/edrone0/mavros/param/set', mavros_msgs.srv.SetMode)
            change_mode_0("COM_RCL_EXCEPT", 4)

            # change_mode_1 = rospy.ServiceProxy(
            #     '/edrone1/mavros/param/set', mavros_msgs.srv.SetMode)
            # change_mode_1("COM_RCL_EXCEPT", 4)
        except rospy.ServiceException as e:
            print("Service param failed %s" % e)

    def setArm_0(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')
        # rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService_0 = rospy.ServiceProxy(
                '/edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_0(True)

        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def setArm_1(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')
        # rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        try:

            armService_1 = rospy.ServiceProxy(
                 '/edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_1(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)


    def offboard_set_mode_0(self):

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:

            set_ModeService_0 = rospy.ServiceProxy(
                'edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
            set_ModeService_0(custom_mode="OFFBOARD")


        except rospy.ServiceException as e:
            print("Service setting mode call failed: %s" % e)
# --
    def offboard_set_mode_1(self):

    #   # Call /mavros/set_mode to set the mode the drone to OFFBOARD
    #   # and print fail message on failure
         rospy.wait_for_service('/edrone1/mavros/set_mode')
         try:

             set_ModeService_1 = rospy.ServiceProxy(
                 'edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
             set_ModeService_1(custom_mode="OFFBOARD")

         except rospy.ServiceException as e:
             print("Service setting mode call failed: %s" % e)

    def setAutoLandMode_0(self):
        rospy.wait_for_service('/edrone0/mavros/set_mode')

        set_ModeService_0 = rospy.ServiceProxy(
            '/edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService_0(custom_mode='AUTO.LAND')
        print("inside autoland")
        # except rospy.ServiceException as e:
        #print ("service set_mode call failed: %s. Autoland Mode could not be set" % e)
# --
    def setAutoLandMode_1(self):
         rospy.wait_for_service('/edrone1/mavros/set_mode')

         set_ModeService_1 = rospy.ServiceProxy(
             '/edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
         set_ModeService_1(custom_mode='AUTO.LAND')

         print("inside autoland")

    def gripper_activate_0(self, grip_control):
        rospy.wait_for_service('/edrone0/activate_gripper')
        gripper = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function_0")
# --

    def gripper_activate_1(self, grip_control):
        rospy.wait_for_service('/edrone0/activate_gripper')
        gripper = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function_1")


class stateMoniter:
    def __init__(self):
        self.state_0 = State()
        self.pos_0 = PositionTarget()
        self.local_pos_0 = Point(0, 0, 0)

        self.state_1 = State()
        self.pos_1 = PositionTarget()
        self.local_pos_1 = Point(-1, 61, 0)

    def stateCb_0(self, msg):
        # Callback function for topic /mavros/state
        self.state_0 = msg

    def stateCb_1(self, msg):
        # Callback function for topic /mavros/state
        self.state_1 = msg

    def posCb_0(self, msg):
        self.local_pos_0.x = msg.pose.position.x
        self.local_pos_0.y = msg.pose.position.y
        self.local_pos_0.z = msg.pose.position.z

    def posCb_1(self, msg):
        self.local_pos_1.x = msg.pose.position.x
        self.local_pos_1.y = msg.pose.position.y
        self.local_pos_1.z = msg.pose.position.z

    # Create more callback functions for other subscribers
    def gripper_check_clbk_0(self, msg):
        self.check_gripper = msg.data
    
    def gripper_check_clbk_1(self, msg):
        self.check_gripper = msg.data
        # rospy.loginfo(self.check_gripper)


class image_processing:

    def __init__(self):

        self.img = np.empty([])
        self.bridge = CvBridge()
        self.eucl_dist = 400
        self.distance_x = 400
        self.distace_y = 400
        self.distance_x_m = 0
        self.distance_y_m = 0
        self.aruco_thresh_bool = False
        self.box_setpoint = list()
        self.pixel_to_meter_ratio = 0
        self.Detected_ArUco_markers = {}

    def detect_ArUco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        Detected_ArUco_markers = {}
        if ids != None:

            Detected_ArUco_markers = dict(zip(ids[:, 0], corners))

        return Detected_ArUco_markers

    def calcuate_centre(self, corners):
        #np_arr = Detected_ArUco_markers[_id]
        np_arr = corners
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]
        #bl = np_arr[0, 3]

        pixel_gap = abs(tl[0]-tr[0])
        if pixel_gap == 0:
            pixel_gap = 10
        # finding pixel to metre conversion ratio
        self.pixel_to_meter_ratio = 0.23/pixel_gap
        #print('pixel_to_meter_ratio', self.pixel_to_meter_ratio)
        print('')

        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return self.ctr

    def image_callback_0(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 200), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame_0', img_2)
            cv2.waitKey(1)
            self.Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(
                    self.Detected_ArUco_markers[key])
                self.distance_x = self.centre[0]-200
                self.distance_y = self.centre[1]-200
                self.eucl_dist = math.sqrt(
                    ((200-self.centre[0])**2)+((200-self.centre[1])**2))
                self.distance_x_m = self.distance_x*self.pixel_to_meter_ratio
                self.distance_y_m = self.distance_y*self.pixel_to_meter_ratio
                #print(self.distance_y_m, 'This is y distance error in meters')
                print("distance is of ArUco id ", key, 'is ', self.eucl_dist,
                      self.distance_x_m, self.distance_y_m)
                # print(self.centre)
        except CvBridgeError as e:
            print(e)
            return

    def image_callback_1(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 200), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame_1', img_2)
            cv2.waitKey(1)
            self.Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(
                    self.Detected_ArUco_markers[key])
                self.distance_x = self.centre[0]-200
                self.distance_y = self.centre[1]-200
                self.eucl_dist = math.sqrt(
                    ((200-self.centre[0])**2)+((200-self.centre[1])**2))
                self.distance_x_m = self.distance_x*self.pixel_to_meter_ratio
                self.distance_y_m = self.distance_y*self.pixel_to_meter_ratio
                #print(self.distance_y_m, 'This is y distance error in meters')
                print("distance is of ArUco id ", key, 'is ', self.eucl_dist,
                      self.distance_x_m, self.distance_y_m)
                # print(self.centre)
        except CvBridgeError as e:
            print(e)
            return


def drone_0():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()

    # Initialize publishers
    local_pos_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints_0 = [(-1, 1, 3), (-1, 16, 3), (13, 16, 3), (13.85, -7.4, 3)]  # List to setpoints
    
    # Similarly initialize other publishers

    # Create empty message containers
    pos_0 = PoseStamped()
    pos_0.pose.position.x = 0
    pos_0.pose.position.y = 0
    pos_0.pose.position.z = 0

    # Set your velocity here
    vel_0 = TwistStamped()
    vel_0.twist.linear.x = 0
    vel_0.twist.linear.y = 0
    vel_0.twist.linear.z = 0



    # Similarly add other containers
    box_setpoint = []

    # Initialize subscriber
    rospy.Subscriber("/edrone0/mavros/state", State, stateMt.stateCb_0)

    rospy.Subscriber("/edrone0/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_0)
    rospy.Subscriber('/edrone0/gripper_check', String,
                     stateMt.gripper_check_clbk_0)

    rospy.Subscriber("/edrone0/camera/image_raw",
                     Image, img_proc.image_callback_0)


    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    def dummy_points_0():
        for i in range(100):
            print('Sending dummy points')
            local_pos_pub_0.publish(pos_0)
            rate.sleep()
    dummy_points_0()
    
    # Arming the drone
    while not stateMt.state_0.armed:
        ofb_ctl.setArm_0()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state_0.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode_0()
        rate.sleep()
    print("OFFBOARD mode activated")
    i = 0

    pos_0.pose.position.x = setpoints_0[i][0]
    pos_0.pose.position.y = setpoints_0[i][1]
    pos_0.pose.position.z = setpoints_0[i][2]



    def check_position_0():
        desired = np.array((setpoints_0[i][0], setpoints_0[i][1], setpoints_0[i][2]))
        pos = np.array((stateMt.local_pos_0.x,
                        stateMt.local_pos_0.y,
                        stateMt.local_pos_0.z))
        print(np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.1




    # Publish the setpoints
    land_count = 0  # for land count
    while not rospy.is_shutdown():

        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here

        '''

        stateMt
        ofb_ctl.setArm_0
        ofb_ctl.offboard_set_mode_0
        reached = check_position_0()
        if (reached == True) and (i <= 4):
            i += 1
            print(i)
            if i==5:
               offboard_control.land_mode
               break
            
            

        pos_0.pose.position.x = setpoints_0[i][0]
        pos_0.pose.position.y = setpoints_0[i][1]
        pos_0.pose.position.z = setpoints_0[i][2]

        local_pos_pub_0.publish(pos_0)
        local_vel_pub_0.publish(vel_0)

        
        

        
        

        rate.sleep()

def drone_1():
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()
    local_pos_pub_1 = rospy.Publisher(
         '/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_1 = rospy.Publisher(
         '/edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate
    rospy.Subscriber("/edrone1/mavros/state", State, stateMt.stateCb_1)

    rospy.Subscriber("/edrone1/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_1)
    rospy.Subscriber('/edrone1/gripper_check', String,
                      stateMt.gripper_check_clbk_1)

    rospy.Subscriber("/edrone1/camera/image_raw",
                      Image, img_proc.image_callback_1)
    rate = rospy.Rate(20.0)

    setpoints_1 = [(-1, 61, 3), (-1, 52, 3), (13, 52, 3), (56, 64, 3)]

    pos_1 = PoseStamped()
    pos_1.pose.position.x = 0
    pos_1.pose.position.y = 0
    pos_1.pose.position.z = 0

    # Set your velocity here
    vel_1 = TwistStamped()
    vel_1.twist.linear.x = 0
    vel_1.twist.linear.y = 0
    vel_1.twist.linear.z = 0

    def dummy_points_1():
        for i in range(100):
            print('Sending dummy points')
            local_pos_pub_1.publish(pos_1)
            rate.sleep()
    dummy_points_1()
    while not stateMt.state_1.armed:
        ofb_ctl.setArm_1()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state_1.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode_0()
        ofb_ctl.offboard_set_mode_1()

        rate.sleep()
    print("OFFBOARD mode activated")
    j = 0
    pos_1.pose.position.x = setpoints_1[j][0]
    pos_1.pose.position.y = setpoints_1[j][1]
    pos_1.pose.position.z = setpoints_1[j][2]


    
    def check_position_1():
        desired = np.array((setpoints_1[j][0], setpoints_1[j][1], setpoints_1[j][2]))
        pos = np.array((stateMt.local_pos_1.x,
                        stateMt.local_pos_1.y,
                        stateMt.local_pos_1.z))
        print(np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.1

 
    # Publish the setpoints
    land_count = 0  # for land count
    while not rospy.is_shutdown():

        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here

        '''

        stateMt
        ofb_ctl.setArm_1
        ofb_ctl.offboard_set_mode_1
        reached = check_position_1()
        if (reached == True) and (j <= 4):
            j += 1
            print(j)
            if j==5:
               offboard_control.land_mode
               break
            
            

        pos_1.pose.position.x = setpoints_1[j][0]
        pos_1.pose.position.y = setpoints_1[j][1]
        pos_1.pose.position.z = setpoints_1[j][2]

        local_pos_pub_1.publish(pos_1)
        local_vel_pub_1.publish(vel_1)

        rate.sleep()

if __name__ == '__main__':

    try:
        p1 = Process(target = drone_0)
        p1.start()
        p2 = Process(target = drone_1)
        p2.start()
    # This is where I had to add the join() function.
        p1.join()
        p2.join()
    except rospy.ROSInterruptException:
        pass
