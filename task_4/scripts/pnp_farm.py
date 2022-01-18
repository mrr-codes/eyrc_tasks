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

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('/edrone0/mavros/cmd/arming')
        # rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService_0 = rospy.ServiceProxy(
                '/edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_0(True)

            # armService_1 = rospy.ServiceProxy(
            #     '/edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            # armService_1(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies

    def offboard_set_mode(self):

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('/edrone0/mavros/set_mode')
        try:

            set_ModeService_0 = rospy.ServiceProxy(
                'edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
            set_ModeService_0(custom_mode="OFFBOARD")

            # set_ModeService_1 = rospy.ServiceProxy(
            #     'edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            # set_ModeService_1(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print("Service setting mode call failed: %s" % e)
# --
    # def offboard_set_mode_1(self):

    #   # Call /mavros/set_mode to set the mode the drone to OFFBOARD
    #   # and print fail message on failure
    #     rospy.wait_for_service('/edrone1/mavros/set_mode')
    #     try:

    #         set_ModeService_1 = rospy.ServiceProxy(
    #             'edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
    #         set_ModeService_1(custom_mode="OFFBOARD")

    #     except rospy.ServiceException as e:
    #         print("Service setting mode call failed: %s" % e)

    def setAutoLandMode(self):
        rospy.wait_for_service('/edrone0/mavros/set_mode')

        set_ModeService_0 = rospy.ServiceProxy(
            '/edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService_0(custom_mode='AUTO.LAND')
        print("inside autoland")
        # except rospy.ServiceException as e:
        #print ("service set_mode call failed: %s. Autoland Mode could not be set" % e)
# --
    # def setAutoLandMode_1(self):
    #     rospy.wait_for_service('/edrone1/mavros/set_mode')

    #     set_ModeService_1 = rospy.ServiceProxy(
    #         '/edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
    #     set_ModeService_1(custom_mode='AUTO.LAND')

    #     print("inside autoland")

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
        self.local_pos_0 = Point(-1, 1, 0)

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
    def gripper_check_clbk(self, msg):
        self.check_gripper = msg.data
        # rospy.loginfo(self.check_gripper)


class image_processing:

    def __init__(self):

        self.img = np.empty([])
        self.bridge = CvBridge()
        self.eucl_dist = 400
        self.distance_x = 1000
        self.aruco_thresh_bool = False
        self.box_setpoint = list()

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
        #tr = np_arr[0, 1]
        br = np_arr[0, 2]
        #bl = np_arr[0, 3]

        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return self.ctr

    def image_callback(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 200), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame', img_2)
            cv2.waitKey(1)
            Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(Detected_ArUco_markers[key])
                # math.sqrt(((200)-(self.centre[0]))**2)
                self.distance_x = self.centre[0]-200.0
                # math.sqrt(((200)-(self.centre[1]))**2)
                distance_y = self.centre[1]-200.0
                self.eucl_dist = math.sqrt(
                    ((200-self.centre[0])**2)+((200-self.centre[1])**2))
                print('Aruco Centre at ', self.centre)
                print("distance is", 'Eucl dist.-', self.eucl_dist,
                      self.distance_x, distance_y)

        except CvBridgeError as e:
            print(e)
            return


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()

    # Initialize publishers
    local_pos_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_0 = rospy.Publisher(
        '/edrone0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    # local_pos_pub_1 = rospy.Publisher(
    #     '/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # local_vel_pub_1 = rospy.Publisher(
    #     '/edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [(0, 0, 3), (9, 0, 3), (9, 0, 3),
                 (0, 0, 3)]  # List to setpoints

    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 3
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0

    # Similarly add other containers
    box_setpoint = []

    # Initialize subscriber
    rospy.Subscriber("/edrone0/mavros/state", State, stateMt.stateCb_0)

    rospy.Subscriber("/edrone0/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_0)
    rospy.Subscriber('/edrone0/gripper_check', String,
                     stateMt.gripper_check_clbk)

    rospy.Subscriber("/edrone0/camera/image_raw",
                     Image, img_proc.image_callback)

    # rospy.Subscriber("/edrone1/mavros/state", State, stateMt.stateCb_1)

    # rospy.Subscriber("/edrone1/mavros/local_position/pose",
    #                  PoseStamped, stateMt.posCb_1)
    # rospy.Subscriber('/edrone1/gripper_check', String,
    #                  stateMt.gripper_check_clbk)

    # rospy.Subscriber("/edrone1/camera/image_raw",
    #                  Image, img_proc.image_callback)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    def dummy_points():
        for i in range(100):
            #print('Sending dummy points')
            local_pos_pub_0.publish(pos)
            rate.sleep()
    dummy_points()
    ofb_ctl.paramset
    # Arming the drone
    while not stateMt.state_0.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state_0.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print("OFFBOARD mode activated")
    i = 0
    pos.pose.position.x = setpoints[i][0]
    pos.pose.position.y = setpoints[i][1]
    pos.pose.position.z = setpoints[i][2]

    def check_position():
        desired = np.array((setpoints[i][0], setpoints[i][1], setpoints[i][2]))
        pos = np.array((stateMt.local_pos_0.x,
                        stateMt.local_pos_0.y,
                        stateMt.local_pos_0.z))
        print(np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.5

    # Publish the setpoints
    c = 0  # for land count
    while not rospy.is_shutdown():

        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here

        '''

        stateMt
        ofb_ctl.setArm()

        if img_proc.distance_x < 150.0:
            # img_proc.box_setpoint = [stateMt.local_pos.x,
            #                          stateMt.local_pos.y, stateMt.local_pos.z]
            #print('Box is at ', img_proc.box_setpoint)
            vel.twist.linear.x = 2
            if img_proc.distance_x < 0:
                img_proc.aruco_thresh_bool = True

        if img_proc.aruco_thresh_bool == True and c == 0:

            # pos.pose.position.x = img_proc.box_setpoint[0]
            # pos.pose.position.y = img_proc.box_setpoint[1]
            # pos.pose.position.z = img_proc.box_setpoint[2]

            # local_pos_pub.publish(pos)
            # rospy.sleep(5)  # trying to create time for grip
            ofb_ctl.setAutoLandMode()
            c += 1
            print('Attempted to land c=', str(c))
            img_proc.aruco_thresh_bool = False
            rospy.sleep(5)
            # print(reached)
            # if (i < 5):
            #     i = i + 1
            # if c == 1 and stateMt.check_gripper == 'True':
            #     ofb_ctl.gripper_activate(True)
            #     print("gripper_true_inside_if")
            # if c == 2:
            #     # stateMt.gripper_check_clbk()
            #     ofb_ctl.gripper_activate(False)
            #     print("making_gripper_false_inside_if")
        reached = check_position()

        if (reached == True):
            print("At ", i)
            i = i+1

            print('Off to ', i)
            dummy_points()
            ofb_ctl.offboard_set_mode()

        # print(setpoints[i])
        if i == 4:
            break
        pos.pose.position.x = setpoints[i][0]
        pos.pose.position.y = setpoints[i][1]
        pos.pose.position.z = setpoints[i][2]

        if i == 1:
            local_vel_pub_0.publish(vel)
        else:
            local_pos_pub_0.publish(pos)

        rate.sleep()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
