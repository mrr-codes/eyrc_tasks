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
        self.distance_x = 1000
        self.distace_y = 1000
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
        #bl = np_arr[0, 3]

        pixel_gap = abs(tl[0]-tr[0])
        self.pixel_to_meter_ratio = 0.23/pixel_gap

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
                self.distance_x = self.centre[0]-200.0
                self.distance_y = self.centre[1]-225.0
                self.eucl_dist = math.sqrt(
                    ((200-self.centre[0])**2)+((225-self.centre[1])**2))
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
                self.distance_x = self.centre[0]-200.0
                self.distance_y = self.centre[1]-225.0
                self.eucl_dist = math.sqrt(
                    ((200-self.centre[0])**2)+((225-self.centre[1])**2))
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
    setpoints_0 = [(0, 0, 3), (-1, 16, 3), (3, 16, 3), (15.7, -5.94, 3), (15.7, -5.94, 1.7), (-1, 28, 3), (3, 28, 3), (16.55, -5.94, 3), (16.55, -5.94, 1.7), (0, 0, 3)
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
        # if (reached == True) and (i <= 4):
        #     i += 1
        #     print('d0 off to sp.', i)
        #     if i == 10:
        #         offboard_control.setAutoLandMode_0()
        #         break

        # pos_0.pose.position.x = setpoints_0[i][0]
        # pos_0.pose.position.y = setpoints_0[i][1]
        # pos_0.pose.position.z = setpoints_0[i][2]

        # local_pos_pub_0.publish(pos_0)
        # local_vel_pub_0.publish(vel_0)
        if len(img_proc.Detected_ArUco_markers) > 0:
            #print('Aruco marker detected')
            img_proc.aruco_thresh_bool = True
            vel_0.twist.linear.x = ((img_proc.distance_x)
                                    * stateMt.local_pos_0.z)/200
            vel_0.twist.linear.y = - \
                (img_proc.distance_y*stateMt.local_pos_0.z)/200
            print('Box detected, the x and y velocities are:',
                  vel_0.twist.linear.x, vel_0.twist.linear.y)
            vel_0.twist.linear.z = 0
            local_vel_pub_0.publish(vel_0)

            #print('error to image:', img_proc.distance_x, img_proc.distance_y)

            if 0 < img_proc.distance_x < 10:  # and abs(img_proc.distance_y)<20

                flag1 = True

                setpoint_stable = (stateMt.local_pos_0.x,
                                   stateMt.local_pos_0.y, 2)
                setpoints_0.insert(i+1, setpoint_stable)
                i = i+1
                # local_pos_pub_0._0._0.publish(pos)

                if reached == True:

                    img_proc.box_setpoint = [
                        stateMt.local_pos_0.x, stateMt.local_pos_0.y]
                    print('Box is at ', img_proc.box_setpoint)

                    pos_0.pose.position.x = img_proc.box_setpoint[0]
                    pos_0.pose.position.y = img_proc.box_setpoint[1]
                    pos_0.pose.position.z = 1

                    local_pos_pub_0.publish(pos_0)

                    if reached == True:
                        print('In landing loop')
                        rospy.sleep(5)
                        ofb_ctl.setAutoLandMode_0()

                        print('Attempted to land c=', str(land_count))
                        rospy.sleep(8)
                        print("Gripping the box")
                        ofb_ctl.gripper_activate_0(True)

                        if stateMt.check_gripper == 'True':
                            print('The box has beem gripped')
                            land_count += 1

                        else:
                            print('The box can not be gripped')

                        img_proc.aruco_thresh_bool = False
                        dummy_points_0()
                        ofb_ctl.offboard_set_mode_0()
                        setpoint = (stateMt.local_pos_0.x,
                                    stateMt.local_pos_0.y, 3)
                        setpoints_0.insert(i+1, setpoint)
                        i = i+1
                        print('Setting flag1 to false again')
                        flag1 = False
                        # print(setpoints)

                local_pos_pub_0.publish(pos_0)

            if flag1 == False and stateMt.local_pos_0.z > 2.5:
                local_vel_pub_0.publish(vel_0)

        else:
            # dummy_points()
            ofb_ctl.offboard_set_mode_0()

            pos_0.pose.position.x = setpoints_0[i][0]
            pos_0.pose.position.y = setpoints_0[i][1]
            pos_0.pose.position.z = setpoints_0[i][2]

            local_pos_pub_0.publish(pos_0)
            # local_vel_pub.publish(vel)

            if reached == True and flag1 == False:
                print("Reached goal")

                if i == len(setpoints_0):
                    ofb_ctl.setAutoLandMode_0()
                    land_count += 1
                    print('Attempted to land c=', str(land_count))
                    break

                else:
                    i = i+1

            if land_count % 2 == 0 and stateMt.check_gripper == 'True':
                rospy.sleep(5)
                ofb_ctl.gripper_activate_0(False)
                print("Releasing box")

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

    setpoints_1 = [(0, 0, 3), (-1, -12, 3), (3, -12, 3),
                   (58.35, 6.21, 3), (58.35, 6.21, 1.7), (-1, -32, 3), (3, -32, 3), (59.2, 6.21, 3), (59.2, 6.21, 1.7), (0, 0, 3)]

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
        ofb_ctl.offboard_set_mode_0()
        ofb_ctl.offboard_set_mode_1()

        rate.sleep()
    print("d1 OFFBOARD mode activated")
    j = 0
    pos_1.pose.position.x = setpoints_1[j][0]
    pos_1.pose.position.y = setpoints_1[j][1]
    pos_1.pose.position.z = setpoints_1[j][2]

    def check_position_1():
        desired = np.array(
            (setpoints_1[j][0], setpoints_1[j][1], setpoints_1[j][2]))
        pos = np.array((stateMt.local_pos_1.x,
                        stateMt.local_pos_1.y,
                        stateMt.local_pos_1.z))
        print('d1', np.linalg.norm(desired - pos))

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
        if len(img_proc.Detected_ArUco_markers) > 0:
            #print('Aruco marker detected')
            img_proc.aruco_thresh_bool = True
            vel_1.twist.linear.x = ((img_proc.distance_x)
                                    * stateMt.local_pos_1.z)/200
            vel_1.twist.linear.y = - \
                (img_proc.distance_y*stateMt.local_pos_1.z)/200
            print('Box detected, the x and y velocities are:',
                  vel_1.twist.linear.x, vel_1.twist.linear.y)
            vel_1.twist.linear.z = 0
            local_vel_pub_1.publish(vel_1)

            #print('error to image:', img_proc.distance_x, img_proc.distance_y)

            if 0 < img_proc.distance_x < 10:  # and abs(img_proc.distance_y)<20

                flag1 = True

                setpoint_stable = (stateMt.local_pos_1.x,
                                   stateMt.local_pos_1.y, 2)
                setpoints_1.insert(i+1, setpoint_stable)
                i = i+1
                # local_pos_pub.publish(pos)

                if reached == True:

                    img_proc.box_setpoint = [
                        stateMt.local_pos_1.x, stateMt.local_pos_1.y]
                    print('Box is at ', img_proc.box_setpoint)

                    pos_1.pose.position.x = img_proc.box_setpoint[0]
                    pos_1.pose.position.y = img_proc.box_setpoint[1]
                    pos_1.pose.position.z = 1

                    local_vel_pub_1.publish(pos_1)

                    if reached == True:
                        print('In landing loop')
                        rospy.sleep(5)
                        ofb_ctl.setAutoLandMode_1()

                        print('Attempted to land c=', str(land_count))
                        rospy.sleep(8)
                        print("Gripping the box")
                        ofb_ctl.gripper_activate_1(True)

                        if stateMt.check_gripper == 'True':
                            print('The box has beem gripped')
                            land_count += 1

                        else:
                            print('The box can not be gripped')

                        img_proc.aruco_thresh_bool = False
                        dummy_points_1()
                        ofb_ctl.offboard_set_mode_1()
                        setpoint = (stateMt.local_pos_1.x,
                                    stateMt.local_pos_1.y, 3)
                        setpoints_1.insert(i+1, setpoint)
                        i = i+1
                        print('Setting flag1 to false again')
                        flag1 = False
                        # print(setpoints_1)

                local_vel_pub_1.publish(pos_1)

            if flag1 == False and stateMt.local_pos_1.z > 2.5:
                local_vel_pub_1.publish(vel_1)

        else:
            # dummy_points()
            ofb_ctl.offboard_set_mode_1()

            pos_1.pose.position.x = setpoints_1[j][0]
            pos_1.pose.position.y = setpoints_1[j][1]
            pos_1.pose.position.z = setpoints_1[j][2]

            local_vel_pub_1.publish(pos_1)
            # local_vel_pub.publish(vel)

            if reached == True and flag1 == False:
                print("Reached goal")

                if i == len(setpoints_1):
                    ofb_ctl.setAutoLandMode_1()
                    land_count += 1
                    print('Attempted to land c=', str(land_count))
                    break

                else:
                    i = i+1

            if land_count % 2 == 0 and stateMt.check_gripper == 'True':
                rospy.sleep(5)
                ofb_ctl.gripper_activate_1(False)
                print("Releasing box")

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
