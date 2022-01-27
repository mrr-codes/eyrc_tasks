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
        print("d0 inside autoland")
        # except rospy.ServiceException as e:
        #print ("service set_mode call failed: %s. Autoland Mode could not be set" % e)
# --

    def setAutoLandMode_1(self):
        rospy.wait_for_service('/edrone1/mavros/set_mode')

        set_ModeService_1 = rospy.ServiceProxy(
            '/edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService_1(custom_mode='AUTO.LAND')

        print("d1 inside autoland")

    def gripper_activate_0(self, grip_control):
        rospy.wait_for_service('/edrone0/activate_gripper')
        gripper = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function_0")
# --

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
        self.check_gripper_0 = msg.data

    def gripper_check_clbk_1(self, msg):
        self.check_gripper_1 = msg.data
        # rospy.loginfo(self.check_gripper)


class image_processing:

    def __init__(self):

        self.img = np.empty([])
        self.bridge = CvBridge()
        self.eucl_dist = 400
        self.position_aruco_x = 1000
        self.position_aruco_y = 1000
        self.aruco_thresh_bool = False
        self.box_setpoint = list()
        self.Detected_ArUco_markers = []

    def detect_ArUco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        Detected_ArUco_markers_func = {}
        if ids != None:

            Detected_ArUco_markers_func = dict(zip(ids[:, 0], corners))

        return Detected_ArUco_markers_func

    def calcuate_centre(self, corners):
        #np_arr = Detected_ArUco_markers[_id]
        np_arr = corners
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]

        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return self.ctr

    def image_callback_0(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame_0', img_2)
            cv2.waitKey(1)
            self.Detected_ArUco_markers = self.detect_ArUco(self.img)

            for key in self.Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(
                    self.Detected_ArUco_markers[key])

                self.position_aruco_x = self.centre[0]
                self.position_aruco_y = self.centre[1]

                #print(self.distance_y_m, 'This is y distance error in meters')
                '''print("distance is", self.eucl_dist,
                      self.distance_x_m, self.distance_y_m)'''
                # print(self.centre)

        except CvBridgeError as e:
            print(e)
            return

    def image_callback_1(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame_1', img_2)
            cv2.waitKey(1)
            self.Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(
                    self.Detected_ArUco_markers[key])

                self.position_aruco_x = self.centre[0]
                self.position_aruco_y = self.centre[1]

                #print(self.distance_y_m, 'This is y distance error in meters')
                '''print("distance is", self.eucl_dist,
                      self.distance_x_m, self.distance_y_m)'''
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
    setpoints_0 = [(0, 0, 3), (-1, 16, 3), (10, 16, 3), (15.7, -5.94, 3), (15.7, -5.94, 2), (15.7, -5.94, 3), (-1, 24, 3), (23, 24, 3), (16.55, -5.94, 3), (16.55, -5.94, 2), (16.55, -5.94, 3), (0, 0, 3)
                   ]  # List to setpoints

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
            print('d0 Sending dummy points')
            local_pos_pub_0.publish(pos_0)
            rate.sleep()

    dummy_points_0()
    # ofb_ctl.paramset

    # Arming the drone
    while not stateMt.state_0.armed:
        ofb_ctl.setArm_0()
        rate.sleep()
    print("d0 Armed!!")

    # Switching the state to auto mode
    while not stateMt.state_0.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode_0()
        rate.sleep()
    print("d0 OFFBOARD mode activated")
    i = 0

    pos_0.pose.position.x = setpoints_0[i][0]
    pos_0.pose.position.y = setpoints_0[i][1]
    pos_0.pose.position.z = setpoints_0[i][2]

    def check_position_0():
        desired = np.array(
            (setpoints_0[i][0], setpoints_0[i][1], setpoints_0[i][2]))
        pos = np.array((stateMt.local_pos_0.x,
                        stateMt.local_pos_0.y,
                        stateMt.local_pos_0.z))
        print('d0', np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.2

    # Publish the setpoints
    land_count = 0  # for land count
    flag1 = False
    previous_x_error = 0
    previous_y_error = 0
    box_dropped = False

    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.setArm_0()
        ofb_ctl.offboard_set_mode_0()
        reached = check_position_0()
        if i == 7:
            box_dropped = False

        if len(img_proc.Detected_ArUco_markers) > 0 and box_dropped == False:
            #print('Aruco marker detected')
            img_proc.aruco_thresh_bool = True
            vel_0.twist.linear.x = (((img_proc.position_aruco_x - 200)*stateMt.local_pos_0.z)/600 - (
                (img_proc.position_aruco_x - 200) - previous_x_error)/40)
            vel_0.twist.linear.y = -((((img_proc.position_aruco_y - (200 + 80/stateMt.local_pos_0.z))*stateMt.local_pos_0.z)/600) - (
                img_proc.position_aruco_y - (200 + 80/stateMt.local_pos_0.z) - previous_y_error)/40)
            print('d0 Box detected, the x and y velocities are:',
                  vel_0.twist.linear.x, vel_0.twist.linear.y)
            vel_0.twist.linear.z = 0

            # *#local_vel_pub_0.publish(vel_0)

            #print('error to image:', img_proc.distance_x, img_proc.distance_y)

            if 0 < (img_proc.position_aruco_x-200) < 10 and 0 < (-img_proc.position_aruco_y + 225) < 10:

                flag1 = True

                img_proc.box_setpoint = [
                    stateMt.local_pos_0.x, stateMt.local_pos_0.y]
                print('d0 Box is at ', img_proc.box_setpoint)

                print('d0 In landing loop')
                rospy.sleep(5)  # %%%%##
                ofb_ctl.setAutoLandMode_0()
                print('d0 Attempted to land c=', str(land_count))
                rospy.sleep(8)
                print("d0 Gripping the box")
                ofb_ctl.gripper_activate_0(True)
                if stateMt.check_gripper_0 == 'True':
                    print('d0 The box has been gripped')
                    land_count += 1
                else:
                    print('d0 The box can not be gripped')

                img_proc.aruco_thresh_bool = False
                dummy_points_0()
                ofb_ctl.setArm_0()
                ofb_ctl.offboard_set_mode_0()
                setpoint = (stateMt.local_pos_0.x,
                            stateMt.local_pos_0.y, 3)
                setpoints_0.insert(i+1, setpoint)
                i = i+1
                print('d0 Setting flag1 to false again')
                flag1 = False
                # print(setpoints)

                local_pos_pub_0.publish(pos_0)

            if flag1 == False and stateMt.local_pos_0.z > 2.5 and stateMt.check_gripper_0 == 'False':
                local_vel_pub_0.publish(vel_0)

            previous_x_error = img_proc.position_aruco_x - 200
            previous_y_error = img_proc.position_aruco_y - \
                (200 + 80/stateMt.local_pos_0.z)

        elif img_proc.aruco_thresh_bool == False:
            # dummy_points()
            ofb_ctl.offboard_set_mode_0()

            pos_0.pose.position.x = setpoints_0[i][0]
            pos_0.pose.position.y = setpoints_0[i][1]
            pos_0.pose.position.z = setpoints_0[i][2]

            local_pos_pub_0.publish(pos_0)
            # local_vel_pub.publish(vel)

            if reached == True and flag1 == False:
                print("d0 Reached goal")

                if i == len(setpoints_0):
                    ofb_ctl.setAutoLandMode_0()
                    land_count += 1
                    print('d0 Attempted to land c=', str(land_count))
                    break

                else:
                    i = i+1

            if i == 5 or i == 11:
                # rospy.sleep(5)
                ofb_ctl.gripper_activate_0(False)
                box_dropped = True
                print("d0 Releasing box")

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

    setpoints_1 = [(0, 0, 3), (-1, -12, 3), (18, -12, 3),
                   (58.35, 6.21, 4), (58.35, 6.21, 2), (58.35, 6.21, 4), (-1, -32, 3), (8, -32, 3), (59.2, 6.21, 4), (59.2, 6.21, 2), (59.2, 6.21, 4), (0, 0, 3)]

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
            print('d1 Sending dummy points')
            local_pos_pub_1.publish(pos_1)
            rate.sleep()
    dummy_points_1()
    while not stateMt.state_1.armed:
        ofb_ctl.setArm_1()
        rate.sleep()
    print("d1 Armed!!")

    # Switching the state to auto mode
    while not stateMt.state_1.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode_1()
        rate.sleep()
    print("d1 OFFBOARD mode activated")
    i = 0
    pos_1.pose.position.x = setpoints_1[i][0]
    pos_1.pose.position.y = setpoints_1[i][1]
    pos_1.pose.position.z = setpoints_1[i][2]

    def check_position_1():
        desired = np.array(
            (setpoints_1[i][0], setpoints_1[i][1], setpoints_1[i][2]))
        pos = np.array((stateMt.local_pos_1.x,
                        stateMt.local_pos_1.y,
                        stateMt.local_pos_1.z))
        print('d1', np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.2

    # Publish the setpoints
    land_count = 0  # for land count
    flag1 = False
    previous_x_error = 0
    previous_y_error = 0
    box_dropped = False

    while not rospy.is_shutdown():

        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here

        '''

        stateMt
        ofb_ctl.setArm_1()
        ofb_ctl.offboard_set_mode_1()
        reached = check_position_1()

        if i == 7:
            box_dropped = False

        if len(img_proc.Detected_ArUco_markers) > 0 and box_dropped == False:
            #print('Aruco marker detected')
            img_proc.aruco_thresh_bool = True
            vel_1.twist.linear.x = (((img_proc.position_aruco_x - 200)*stateMt.local_pos_1.z)/600 - (
                (img_proc.position_aruco_x - 200) - previous_x_error)/40)
            vel_1.twist.linear.y = -((((img_proc.position_aruco_y - (200 + 80/stateMt.local_pos_1.z))*stateMt.local_pos_1.z)/600) - (
                img_proc.position_aruco_y - (200 + 80/stateMt.local_pos_1.z) - previous_y_error)/40)
            print('d1 Box detected, the x and y velocities are:',
                  vel_1.twist.linear.x, vel_1.twist.linear.y)
            vel_1.twist.linear.z = 0
            # local_vel_pub_1.publish(vel_1)

            #print('error to image:', img_proc.distance_x, img_proc.distance_y)

            if 0 < (img_proc.position_aruco_x-200) < 10 and 0 < (-img_proc.position_aruco_y + 225) < 10:

                flag1 = True

                img_proc.box_setpoint = [
                    stateMt.local_pos_1.x, stateMt.local_pos_1.y]
                print('d1 Box is at ', img_proc.box_setpoint)

                print('d1 In landing loop')
                rospy.sleep(5)
                ofb_ctl.setAutoLandMode_1()
                print('d1 Attempted to land c=', str(land_count))
                rospy.sleep(8)
                print("d1 Gripping the box")
                ofb_ctl.gripper_activate_1(True)
                if stateMt.check_gripper_1 == 'True':
                    print('d1 The box has been gripped')
                    land_count += 1
                else:
                    print('d1 The box can not be gripped')

                img_proc.aruco_thresh_bool = False
                dummy_points_1()
                ofb_ctl.setArm_1()
                ofb_ctl.offboard_set_mode_1()
                setpoint = (stateMt.local_pos_1.x,
                            stateMt.local_pos_1.y, 3)
                setpoints_1.insert(i+1, setpoint)
                i = i+1
                print('d1 Setting flag1 to false again')
                flag1 = False
                # print(setpoints)

                local_pos_pub_1.publish(pos_1)

            if flag1 == False and stateMt.local_pos_1.z > 2.5 and stateMt.check_gripper_1 == 'False':
                local_vel_pub_1.publish(vel_1)

            previous_x_error = img_proc.position_aruco_x - 200
            previous_y_error = img_proc.position_aruco_y - \
                (200 + 80/stateMt.local_pos_1.z)

        elif img_proc.aruco_thresh_bool == False:
            # dummy_points()
            ofb_ctl.offboard_set_mode_1()

            pos_1.pose.position.x = setpoints_1[i][0]
            pos_1.pose.position.y = setpoints_1[i][1]
            pos_1.pose.position.z = setpoints_1[i][2]

            local_pos_pub_1.publish(pos_1)
            # local_vel_pub.publish(vel)

            if reached == True and flag1 == False:
                print("d1 Reached goal")

                if i == len(setpoints_1):
                    ofb_ctl.setAutoLandMode_1()
                    land_count += 1
                    print('d1 Attempted to land c=', str(land_count))
                    break

                else:
                    i = i+1

            if i == 5 or i == 11:
                rospy.sleep(5)
                ofb_ctl.gripper_activate_1(False)
                box_dropped = True
                print("d1 Releasing box")

        rate.sleep()


if __name__ == '__main__':

    try:
        p1 = Process(target=drone_0)
        p1.start()
        p2 = Process(target=drone_1)
        p2.start()
    # This is where I had to add the join() function.
        p1.join()
        p2.join()
    except rospy.ROSInterruptException:
        pass
