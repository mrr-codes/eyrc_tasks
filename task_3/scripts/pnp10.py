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
        


class image_processing:

    def __init__(self):

        self.img = np.empty([])
        self.bridge = CvBridge()
        self.eucl_dist = 400
        self.position_aruco_x = 1000
        self.position_aruco_y = 1000
        self.aruco_thresh_bool = False
        self.box_setpoint = list()
        self.pixel_to_meter_ratio = 0
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
        #bl = np_arr[0, 3]

        pixel_gap = abs(tl[0]-tr[0])
        self.pixel_to_meter_ratio = 0.23/pixel_gap

        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return self.ctr

    def image_callback(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            img_2 = cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)
            cv2.imshow('check_frame', img_2)
            cv2.waitKey(1)
            self.Detected_ArUco_markers = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers.keys():
                self.centre = self.calcuate_centre(self.Detected_ArUco_markers[key])

                self.position_aruco_x = self.centre[0]
                self.position_aruco_y = self.centre[1]
 

                #print(self.distance_y_m, 'This is y distance error in meters')
                '''print("distance is", self.eucl_dist,
                      self.distance_x_m, self.distance_y_m)'''
                # print(self.centre)


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
        'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [(0, 0, 3), (9, 0, 3), (9, 0, 3),(0, 0, 3)]  # List to setpoints

    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0

    # Similarly add other containers
    box_setpoint = []

    # Initialize subscriber
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.posCb)
    rospy.Subscriber('/gripper_check', String, stateMt.gripper_check_clbk)

    rospy.Subscriber("/iris/camera/image_raw",
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
    pos.pose.position.x = setpoints[i][0]
    pos.pose.position.y = setpoints[i][1]
    pos.pose.position.z = setpoints[i][2]

    def check_position():
        desired = np.array((setpoints[i][0], setpoints[i][1], setpoints[i][2]))
        pos = np.array((stateMt.local_pos.x,
                        stateMt.local_pos.y,
                        stateMt.local_pos.z))
        #print('error to desired position:',np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.1

    land_count = 0  # for land count
    flag1 = False
    previous_x_error=0
    previous_y_error=0



    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.setArm()
        reached = check_position()
        #print(img_proc.Detected_ArUco_markers)

        if len(img_proc.Detected_ArUco_markers) > 0:
            #print('Aruco marker detected')
            img_proc.aruco_thresh_bool = True
            vel.twist.linear.x = (((img_proc.position_aruco_x - 200)*stateMt.local_pos.z)/600  - ((img_proc.position_aruco_x - 200)- previous_x_error)/40)                         
            vel.twist.linear.y = -(((   (img_proc.position_aruco_y - (200 + 80/stateMt.local_pos.z)   )*stateMt.local_pos.z)/600) - (img_proc.position_aruco_y - (200 + 80/stateMt.local_pos.z)- previous_y_error)/40) 
            print('Box detected, the x and y velocities are:',vel.twist.linear.x, vel.twist.linear.y)
            vel.twist.linear.z = 0
            #local_vel_pub.publish(vel)

                      


            if 0<(img_proc.position_aruco_x-200)<10: #and abs(img_proc.distance_y)<20

               
                flag1 = True 

                # setpoint_stable=(stateMt.local_pos.x,stateMt.local_pos.y,2) 
                # setpoints.insert(i+1,setpoint_stable)
                # i=i+1
                # #local_pos_pub.publish(pos)
                               

                # if reached == True :                                
                
                img_proc.box_setpoint = [stateMt.local_pos.x , stateMt.local_pos.y]
                print('Box is at ', img_proc.box_setpoint)

                # pos.pose.position.x = img_proc.box_setpoint[0]
                # pos.pose.position.y = img_proc.box_setpoint[1]
                # pos.pose.position.z = 1
                
                # local_pos_pub.publish(pos)

                # if reached == True:
                print('In landing loop')
                rospy.sleep(5)
                ofb_ctl.setAutoLandMode()
                
                print('Attempted to land c=', str(land_count))
                rospy.sleep(8)
                print("Gripping the box")
                ofb_ctl.gripper_activate(True)

                if stateMt.check_gripper == 'True':
                    print('The box has beem gripped')
                    land_count += 1
                
                else: print('The box can not be gripped')

                img_proc.aruco_thresh_bool = False
                dummy_points()
                ofb_ctl.offboard_set_mode()
                setpoint=(stateMt.local_pos.x,stateMt.local_pos.y,3) 
                setpoints.insert(i+1,setpoint)
                i=i+1
                print('Setting flag1 to false again')
                flag1 = False
                #print(setpoints)

                local_pos_pub.publish(pos)

            if flag1 == False and stateMt.local_pos.z>2.5 and stateMt.check_gripper == 'False':    
                local_vel_pub.publish(vel)

            previous_x_error = img_proc.position_aruco_x - 200
            previous_y_error = img_proc.position_aruco_y - (200 + 80/stateMt.local_pos.z)




        elif img_proc.aruco_thresh_bool==False:
            #dummy_points()
            ofb_ctl.offboard_set_mode()

            pos.pose.position.x = setpoints[i][0]
            pos.pose.position.y = setpoints[i][1]
            pos.pose.position.z = setpoints[i][2]

            local_pos_pub.publish(pos)
            #local_vel_pub.publish(vel)

            if reached == True and flag1 == False:
                print("Reached goal")

                if i == len(setpoints):
                    ofb_ctl.setAutoLandMode()
                    land_count += 1
                    print('Attempted to land c=', str(land_count))
                    break

                else: 
                    i = i+1



            if land_count%2 == 0 and stateMt.check_gripper == 'True':
                rospy.sleep(5)    
                ofb_ctl.gripper_activate(False)
                print("Releasing box")




        rate.sleep()





if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass
