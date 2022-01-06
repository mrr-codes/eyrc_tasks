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


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)
        # Waiting untill the service starts

    def paramset():
        rospy.wait_for_service('/mavros/param/set')
        try:
            change_mode = rospy.ServiceProxy(
                '/mavros/param/set', mavros_msgs.srv.SetMode)
            change_mode("COM_RCL_EXCEPT", 4)
        except rospy.ServiceException as e:
            print("Service param failed %s" % e)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies

    def offboard_set_mode(self):

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:

            set_ModeService = rospy.ServiceProxy(
                'mavros/set_mode', mavros_msgs.srv.SetMode)
            set_ModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print("Service setting mode call failed: %s" % e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')

        set_ModeService = rospy.ServiceProxy(
            'mavros/set_mode', mavros_msgs.srv.SetMode)
        set_ModeService(custom_mode='AUTO.LAND')
        print("inside autoland")
        # except rospy.ServiceException as e:
        #print ("service set_mode call failed: %s. Autoland Mode could not be set" % e)

    def gripper_activate(self, grip_control):
        rospy.wait_for_service('/activate_gripper')
        gripper = rospy.ServiceProxy('/activate_gripper', Gripper)
        gripper(grip_control)
        print("gripper_activated_function")


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.pos = PositionTarget()
        self.local_pos = Point(0, 0, 0)

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    # Create more callback functions for other subscribers
    def gripper_check_clbk(self, msg):
        self.check_gripper = msg.data
        rospy.loginfo(self.check_gripper)


class image_processing():

    def __init__(self):

        # # Making a publisher
        # self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)

        # # Subscribing to /eDrone/camera/image_raw
        # self.image_sub = rospy.Subscriber(
        #     "/eDrone/camera/image_raw", Image, self.image_callback)

        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()

        # # This will contain the message structure of message type task_1/Marker
        # self.marker_msg = Marker()

        #self.rate = rospy.Rate(10)

    def detect_ArUco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        # argument: img is the test image
        # return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
        # for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
        # {0: array([[315, 163],
        #							[319, 263],
        #							[219, 267],
        #							[215,167]], dtype=float32)}
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

        ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return ctr

    def image_callback(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 200), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame', img_2)
            cv2.waitKey(1)
            Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in Detected_ArUco_markers.keys():
                centre = self.calcuate_centre(Detected_ArUco_markers[key])

            print('ArUco center is', centre)
        except CvBridgeError as e:
            print(e)
            return


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [(0, 0, 0), (9, 0, 3)]  # List to setpoints

    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0

    # Similarly add other containers

    # Initialize subscriber
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check', String, stateMt.gripper_check_clbk)

    rospy.Subscriber("/eDrone/camera/image_raw",
                     Image, img_proc.image_callback)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    def dummy_points():
        for i in range(100):
            #print('Sending dummy points')
            local_pos_pub.publish(pos)
            rate.sleep()
    dummy_points()
    ofb_ctl.paramset
    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print("OFFBOARD mode activated")
    i = 0
    # pos.pose.position.x = setpoints[i][0]
    # pos.pose.position.y = setpoints[i][1]
    # pos.pose.position.z = setpoints[i][2]

    def check_position():
        desired = np.array((setpoints[i][0], setpoints[i][1], setpoints[i][2]))
        pos = np.array((stateMt.local_pos.x,
                        stateMt.local_pos.y,
                        stateMt.local_pos.z))
        #print(np.linalg.norm(desired - pos))

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

        reached = check_position()

        # if reached == True and (i == 1 or i == 3 or i == 5):

        #     ofb_ctl.setAutoLandMode()
        #     c += 1
        #     rospy.sleep(6)  # trying to create time for grip
        #     print('Attempted to land c=', str(c))
        #     # print(reached)
        #     if (i <= 5):
        #         # if (i < 5):
        #         #     i = i + 1
        #         if c == 1 and stateMt.check_gripper == 'True':
        #             ofb_ctl.gripper_activate(True)
        #             print("gripper_true_inside_if")
        #         if c == 2:
        #             # stateMt.gripper_check_clbk()
        #             ofb_ctl.gripper_activate(False)
        #             print("making_gripper_false_inside_if")

        if (reached == True):
            print("off", i)
            #i = i+1

            print(i)
            dummy_points()
            ofb_ctl.offboard_set_mode()

        # print(setpoints[i])
        # if i == 2:
        #     break
        pos.pose.position.x = setpoints[i][0]
        pos.pose.position.y = setpoints[i][1]
        pos.pose.position.z = setpoints[i][2]

        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)

        rate.sleep()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
