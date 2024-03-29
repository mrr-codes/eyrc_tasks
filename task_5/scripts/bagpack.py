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
        rospy.init_node('offboard_control', anonymous=True)


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

    def setArm_1(self):
        rospy.wait_for_service('/edrone1/mavros/cmd/arming')
        print('setArm_1 called')
        try:
            armService_1 = rospy.ServiceProxy(
                '/edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService_1(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

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
        self.row_spawn_sp0 = list()
        self.row_spawn_sp1 = list()
        self.spawn_count = 0
        self.row_list = list()
        # self.bt_i = -1  # neeed to write reset condition
        # self.rt_i = -1
        self.bt_0 = -1  # neeed to write reset condition
        self.rt_0 = -1
        self.bt_1 = 0
        self.rt_1 = 0
        self.box_counts = dict()

        self.blue_truck = np.array([[(13.85, -7.4, 1.84), (13.85, -6.17, 1.84), (13.85, -4.95, 1.84)], [(14.7, -7.4, 1.84), (14.7, -6.17, 1.84),
                                   (14.7, -4.95, 1.84)], [(15.55, -7.4, 1.84), (15.55, -6.17, 1.84), (15.55, -4.95, 1.84)], [(16.4, -7.4, 1.84), (16.4, -6.17, 1.84), (16.4, -4.95, 1.84)]])

        self.red_truck = np.array([[(56.5, 64.75, 1.84), (56.5, 65.98, 1.84), (56.5, 67.21, 1.84)], [(57.35, 64.75, 1.84), (57.35, 65.98, 1.84), (57.35, 67.21, 1.84)],
                                  [(58.2, 64.75, 1.84), (58.2, 65.98, 1.84), (58.2, 67.21, 1.84)], [(59.05, 64.75, 1.84), (59.05, 65.98, 1.84), (59.05, 67.21, 1.84)]])

        # self.blue_truck_seq = [self.blue_truck[1, 0],self.blue_truck[2, 1],self.blue_truck[1,2],
        # self.blue_truck[3,0],self.blue_truck[1,1],self.blue_truck[3,1],self.blue_truck[2,2],self.blue_truck[2,0],self.blue_truck[2,1]]

        # self.red_truck_seq = [self.red_truck[1, 0],self.red_truck[2, 1],self.red_truck[1,2],
        # self.red_truck[3,0],self.red_truck[1,1],self.red_truck[3,1],self.red_truck[2,2],self.red_truck[2,0],self.red_truck[2,1]]

        self.blue_truck_seq = [self.blue_truck[3, 2], self.blue_truck[3, 1], self.blue_truck[3, 0], self.blue_truck[2, 0], self.blue_truck[2, 1], self.blue_truck[
            2, 2], self.blue_truck[1, 2], self.blue_truck[1, 1], self.blue_truck[1, 0]]

        self.red_truck_seq = [self.red_truck[3, 2], self.red_truck[3, 1], self.red_truck[3, 0], self.red_truck[2, 0], self.red_truck[2, 1], self.red_truck[
            2, 2], self.red_truck[1, 2], self.red_truck[1, 1], self.red_truck[1, 0]]



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

    def calculate_row_start(self, row_no, drone_no):

        # if drone_no == 0:
        #     if row_no in self.row_list:
        #         self.row_list.append(row_no)
        #         return (5, 4*(row_no-1), 3)
        #     else:
        #         self.row_list.append(row_no)
        #         return(0,4*(row_no-1),3)
        # else:
        #     if row_no in self.row_list:
        #         self.row_list.append(row_no)
        #         return (5, 4*(row_no-16), 3)
        #     else:
        #         self.row_list.append(row_no)
        #         return(0,4*(row_no-16),3)
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
                return(-1.4,4*(row_no-16),3)

    def spawn_clbk(self, msg):
        
        self.box_counts[msg.data] = self.box_counts.get(msg.data, 0) + 1

        if self.spawn_count % 2 == 0:
            self.row_spawn_sp0.append(self.calculate_row_start(msg.data, 0))
            print('d0 Row_spawn list',self.row_spawn_sp0)

        else:
            self.row_spawn_sp1.append(self.calculate_row_start(msg.data, 1))
            print('d1 Row_spawn list',self.row_spawn_sp1)
            
        self.spawn_count += 1

    def calculate_truck_point(self, id, drone_no):
        if id == 2:  # blue
            #self.bt_i += 1
            if drone_no == 0:
                self.bt_0 += 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.blue_truck_seq[self.bt_0], (-1, 1, 0)))
                final_array = [(drop_pt[0], drop_pt[1],5),
                                (drop_pt[0], drop_pt[1],5)]

            else:
                self.bt_1 -= 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.blue_truck_seq[self.bt_1], (-1, 61, 0)))
                final_array = [(drop_pt[0], drop_pt[1], 6),
                                (drop_pt[0], drop_pt[1], 6)]

        else:
            if drone_no == 0:
                self.rt_0 += 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.red_truck_seq[self.rt_0], (-1, 1, 0)))
                final_array = [(drop_pt[0], drop_pt[1], 5), (drop_pt[0], drop_pt[1], 5)]

            else:
                self.rt_1 -= 1
                drop_pt = tuple(
                    map(lambda i, j: i-j, self.red_truck_seq[self.rt_1], (-1, 61, 0)))
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

    def detect_ArUco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        Detected_ArUco_markers_func = {}
        if len(corners) > 0:

            Detected_ArUco_markers_func = dict(zip(ids[:, 0], corners))    

        return Detected_ArUco_markers_func

    def calcuate_centre(self, corners):
        ArUco_marker_angles = {}

        np_arr = corners
        tl = np_arr[0, 0]
        tr = np_arr[0, 1]
        br = np_arr[0, 2]
        bl = np_arr[0, 3]
        self.bcorner_0 = br
        self.bcorner_1 = br

        y = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        diffs = [(tl[0]-bl[0]), (tl[1]-bl[1])]
        degree = math.atan2(diffs[0], diffs[1])*180/math.pi
        final_degree = degree+float(270)
        if (final_degree > 360.0):
            final_degree -= 360.0
        ArUco_marker_angles = final_degree  

        self.ctr = [(tl[0]+br[0])/2, (tl[1]+br[1])/2]
        return self.ctr, ArUco_marker_angles

    def image_callback_0(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)
           
            self.Detected_ArUco_markers_0 = self.detect_ArUco(self.img)

            #print('detected aruco dict is', self.Detected_ArUco_markers_0)
            for key in self.Detected_ArUco_markers_0.keys():
                self.centre, angle_detected = self.calcuate_centre(
                    self.Detected_ArUco_markers_0[key])

                self.position_aruco_x_0 = self.centre[0]
                self.position_aruco_y_0 = self.centre[1]
                self.bcorner_0 = self.bcorner_0

                self.exo_rad_0=math.sqrt((self.bcorner_0[0]-self.position_aruco_x_0)**2+(self.bcorner_0[1]-self.position_aruco_y_0)**2)
                cv2.circle(self.img,(int(self.position_aruco_x_0),int(self.position_aruco_y_0)),radius=abs(int(self.exo_rad_0)),color= (0,225,0),thickness=2)

                samp = str(round(angle_detected))
                cv2.putText(self.img, samp, ((int(self.ctr[0])-90), int(self.ctr[1])),fontFace=cv2.FONT_HERSHEY_COMPLEX, thickness=2, fontScale=0.75, color=(0, 255, 0))
            
            cv2.imshow('check_frame_0', self.img)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return

    def image_callback_1(self, data):

        try:
            self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.circle(self.img, (200, 225), radius=2,
                               color=(0, 0, 255), thickness=-1)

            self.Detected_ArUco_markers_1 = self.detect_ArUco(self.img)
            for key in self.Detected_ArUco_markers_1.keys():
                self.centre, angle_detected = self.calcuate_centre(
                    self.Detected_ArUco_markers_1[key])
                self.position_aruco_x_1 = self.centre[0]
                self.position_aruco_y_1 = self.centre[1]
                self.bcorner_1 = self.bcorner_1
                self.exo_rad_1=math.sqrt((self.bcorner_1[0]-self.position_aruco_x_1)**2+(self.bcorner_1[1]-self.position_aruco_y_1)**2)
                cv2.circle(self.img,(int(self.position_aruco_x_1),int(self.position_aruco_y_1)),radius=abs(int(self.exo_rad_1)),color= (0,225,0),thickness=2)   

                samp = str(round(angle_detected))
                cv2.putText(self.img, samp, ((int(self.ctr[0])-90), int(self.ctr[1])),fontFace=cv2.FONT_HERSHEY_COMPLEX, thickness=2, fontScale=0.75, color=(0, 255, 0))
                          
            cv2.imshow('check_frame_1', self.img)
            cv2.waitKey(1)               
            

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
    
    rospy.Subscriber("/edrone0/mavros/state", State, stateMt.stateCb_0)
    rospy.Subscriber("/edrone0/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_0)
    rospy.Subscriber('/edrone0/gripper_check', String,
                     stateMt.gripper_check_clbk_0)
    rospy.Subscriber("/edrone0/camera/image_raw",
                     Image, img_proc.image_callback_0)
    rospy.Subscriber('/spawn_info', UInt8, stateMt.spawn_clbk)
    rate = rospy.Rate(20.0)

    setpoints_0 = [(0, 0, 3)]

    pos_0 = PoseStamped()
    pos_0.pose.position.x = 0
    pos_0.pose.position.y = 0
    pos_0.pose.position.z = 0

    vel_0 = TwistStamped()
    vel_0.twist.linear.x = 0
    vel_0.twist.linear.y = 0
    vel_0.twist.linear.z = 0
    
    def dummy_points_0():
        for i in range(100):
            print('d0 Sending dummy points')
            local_pos_pub_0.publish(pos_0)
            rate.sleep()
    dummy_points_0()

    def arm_0():
        while not stateMt.state_0.armed:
            ofb_ctl.setArm_0()
            rate.sleep()
        print("d0 Armed!!")
        ofb_ctl.setArm_0()
    arm_0()

    def offboard_0():
        while not stateMt.state_0.mode == "OFFBOARD":
            ofb_ctl.offboard_set_mode_0()
            rate.sleep()
        print("d0 OFFBOARD mode activated")
    offboard_0()

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
        #print('d0', np.linalg.norm(desired - pos))
        if (i > 3 and i == (len(setpoints_0) - 3)) or (i==1):
            return np.linalg.norm(desired - pos) < 0.2
        else:
            return np.linalg.norm(desired - pos) < 0.5
    
    land_count = 0 
    flag1 = False
    previous_x_error = 0
    previous_y_error = 0
    vi = 0
    box_dropped = True
    flag_flip_pos_vol = False
    k = 0
    m = -1
    x = 0
    ofb_ctl.setArm_0()
    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.offboard_set_mode_0()
        reached = check_position_0()

        if i > 4 and (len(setpoints_0)-1):
            print('d0 clearing spts.')
            setpoints_0.clear()
            i = 0
            k += 1
            vi = 0.04
            previous_y_error = 0
            setpoints_0.extend([(stateMt.local_pos_0.x,stateMt.local_pos_0.y,6),stateMt.row_spawn_sp0[k],(0,0,4)])
            print('d0 Setpoints list as of now', setpoints_0)

        if  i==2 and flag_flip_pos_vol == True :
            print(i)
            box_dropped = False
            m = -1
            print("d0 setting m to -1")

        if len(img_proc.Detected_ArUco_markers_0) > 0 and box_dropped == False:

            flag_flip_pos_vol = False
            img_proc.aruco_thresh_bool = True
            
            if (m < 0):
                flag_flip_pos_vol = False
                if 0 < img_proc.position_aruco_x_0 < 250:
                    if stateMt.local_pos_0.x> 3:
                        vi = 0.07
                    print('d0 publishing set pt to decrease height to 1m')
                    pos_0.pose.position.x = stateMt.local_pos_0.x
                    pos_0.pose.position.y = stateMt.row_spawn_sp0[k][1]
                    pos_0.pose.position.z = 1.5 
                    local_pos_pub_0.publish(pos_0)
                    rospy.sleep(5)
                    m += 1
            if m == 0 :
                if (225-img_proc.position_aruco_y_0)<0:
                    vi = 0.1
                else:
                    vi = 0
                vel_0.twist.linear.x = (
                    ((img_proc.position_aruco_x_0 - 200)*stateMt.local_pos_0.z)/300)
                vel_0.twist.linear.y = -((((img_proc.position_aruco_y_0 - (200 + 80/stateMt.local_pos_0.z))*stateMt.local_pos_0.z)/400) - (
                    img_proc.position_aruco_y_0 - (200 + 80/stateMt.local_pos_0.z) - previous_y_error)/40)-vi
                vel_0.twist.linear.z = (1.5-stateMt.local_pos_0.z)/20
        
                local_vel_pub_0.publish(vel_0)
               
            #if ((img_proc.position_aruco_x_0-10) < (200) < (img_proc.position_aruco_x_0+10)) and (img_proc.position_aruco_y_0 -25  < (225) < (img_proc.position_aruco_y_0)):
            if(((200 - img_proc.position_aruco_x_0)**2 + (225-img_proc.position_aruco_y_0)**2)<= (img_proc.exo_rad_0)**2) and ((225 <img_proc.position_aruco_y_0-4) and (190<img_proc.position_aruco_x_0)):
                flag1 = True
                box_id = list(img_proc.Detected_ArUco_markers_0.keys())[0]

                img_proc.box_setpoint = [
                    stateMt.local_pos_0.x, stateMt.local_pos_0.y]
                print('d0 Box is at ', img_proc.box_setpoint)

                pos_0.pose.position.x = img_proc.box_setpoint[0]+0.2
                pos_0.pose.position.y = img_proc.box_setpoint[1]+0.2
                pos_0.pose.position.z = 1
                local_pos_pub_0.publish(pos_0)
                ofb_ctl.setAutoLandMode_0()
                print('d0 Attempted to land c=', str(land_count))
                # rospy.sleep(12)
                # print("d0 Gripping the box")
                # ofb_ctl.gripper_activate_0(True)
                while not stateMt.check_gripper_0 == 'True':
                    if stateMt.local_pos_0.z < 0.25:
                        ofb_ctl.gripper_activate_0(True)
                stateMt.boxes_in_row -= 1  
                if stateMt.check_gripper_0 == 'True':
                    print('d0 The box has been gripped')
                    land_count += 1
                    box_dropped = True
                    img_proc.aruco_thresh_bool = False
                else:
                    print('d0 The box cannot yet be gripped')

               
                # dummy_points_0()
                # arm_0()
                # ofb_ctl.setArm_0()
                offboard_0()
                # ofb_ctl.setArm_0()
                setpoint = (stateMt.local_pos_0.x,
                            stateMt.local_pos_0.y, 3)
                setpoints_0.insert(i+1, setpoint)
                i = i+1
                print('d0 i increased to ', i, 'after re-arming')
                print('d0 Setting flag1 to false again')
                
                local_pos_pub_0.publish(pos_0)
                
                if stateMt.check_gripper_0 == 'True':
                    truck_pts = stateMt.calculate_truck_point(box_id, 0)
                    setpoints_0.extend(
                        [truck_pts[0], truck_pts[1], (0, 0, 0)])
                    print('d0 Setpoints list as of now', setpoints_0)
                    flag1 = False

            #previous_x_error = img_proc.position_aruco_x_0 - 200
            previous_y_error = img_proc.position_aruco_y_0 - \
                (200 + 80/stateMt.local_pos_0.z)

            last_pixel_aruco_detected = img_proc.position_aruco_x_0 

        elif img_proc.aruco_thresh_bool == False:
            ofb_ctl.offboard_set_mode_0()

            pos_0.pose.position.x = setpoints_0[i][0]
            pos_0.pose.position.y = setpoints_0[i][1]
            pos_0.pose.position.z = setpoints_0[i][2]

            if reached == True and  x!= 0 and (abs(setpoints_0[i][1]) % 4 == 0):
                #print('d0 At row start velocity control')
                vel_0.twist.linear.x = 1.5
                vel_0.twist.linear.y = (setpoints_0[i][1] - stateMt.local_pos_0.y)/10
                vel_0.twist.linear.z = 3 - stateMt.local_pos_0.z
                
                flag_flip_pos_vol = True  # have to turn it false in aruco detected

            if flag_flip_pos_vol == True:
                print('d0 publishing row start velocity')
                local_vel_pub_0.publish(vel_0)
                box_dropped = False

            else:

                local_pos_pub_0.publish(pos_0)

            if reached == True and flag1 == False:
                print("d0 Reached goal")
                while x == 0:
                    setpoints_0.extend([stateMt.row_spawn_sp0[k],(0,4,4)])
                    print('d0 after reaching goal setpoints are', setpoints_0)
                    x = x+1

                i = i+1
                print('d0 i increased to ', i, 'after reaching goal')

            if i > 3 and i == (len(setpoints_0) - 2):
                ofb_ctl.setAutoLandMode_0() 
                while not stateMt.local_pos_0.z < 2.3:
                    lol=0
                for o in range(3):
                    ofb_ctl.gripper_activate_0(False)
                    
                box_dropped = True
                print("d0 Releasing box")

            elif img_proc.aruco_thresh_bool == True and len(img_proc.Detected_ArUco_markers_0) == 0:
                if last_pixel_aruco_detected<200:
                    x = -1
                else: x = 1
                
                vel_0.twist.linear.x = x
                vel_0.twist.linear.y = 0
                vel_0.twist.linear.z = 0

                local_vel_pub_0.publish(vel_0)

        rate.sleep()


def drone_1():
    #rospy.sleep(2)
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    img_proc = image_processing()
    local_pos_pub_1 = rospy.Publisher(
        '/edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub_1 = rospy.Publisher(
        '/edrone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    rospy.Subscriber("/edrone1/mavros/state", State, stateMt.stateCb_1)
    rospy.Subscriber("/edrone1/mavros/local_position/pose",
                     PoseStamped, stateMt.posCb_1)
    rospy.Subscriber('/edrone1/gripper_check', String,
                     stateMt.gripper_check_clbk_1)
    rospy.Subscriber("/edrone1/camera/image_raw",
                     Image, img_proc.image_callback_1)
    rospy.Subscriber('/spawn_info', UInt8, stateMt.spawn_clbk)

    rate = rospy.Rate(20.0)

    setpoints_1= [(0, 0, 3)]

    pos_1 = PoseStamped()
    pos_1.pose.position.x = 0
    pos_1.pose.position.y = 0
    pos_1.pose.position.z = 0

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

    def arm_1():
        while not stateMt.state_1.armed:
            ofb_ctl.setArm_1()
            rate.sleep()
        ofb_ctl.setArm_1()
        print("d1 Armed!!")
    arm_1()

    def offboard_1():
        while not stateMt.state_1.mode == "OFFBOARD":
            ofb_ctl.offboard_set_mode_1()
            rate.sleep()
        print("d1 OFFBOARD mode activated")
    offboard_1()

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

        if (i > 3 and i == (len(setpoints_1) - 3)) or (i==1):
            return np.linalg.norm(desired - pos) < 0.2
        else:
            return np.linalg.norm(desired - pos) < 0.5

    
    land_count = 0 
    flag1 = False
    previous_x_error = 0
    previous_y_error = 0
    vi = 0.07
    box_dropped = True
    flag_flip_pos_vol = False
    k = 0
    m = -1
    x = 0
    ofb_ctl.setArm_1()

    while not rospy.is_shutdown():

        stateMt
        ofb_ctl.offboard_set_mode_1()
        reached = check_position_1()

        if i > 4 and (len(setpoints_1)-1):
            print('d1 clearing spts.')
            setpoints_1.clear()
            i = 0
            k += 1
            previous_y_error = 0
            setpoints_1.extend([(stateMt.local_pos_1.x,stateMt.local_pos_1.y,7),stateMt.row_spawn_sp1[k],(0,0,4)])
            print('d1 Setpoints list as of now', setpoints_1)

        if  i==2 and flag_flip_pos_vol == True :
            print(i)
            box_dropped = False
            m = -1
            print("d1 setting m to -1")

        if len(img_proc.Detected_ArUco_markers_1) > 0 and box_dropped == False:
            
            flag_flip_pos_vol = False
            img_proc.aruco_thresh_bool = True

            if (m < 0):
                flag_flip_pos_vol = False
                if 0 < img_proc.position_aruco_x_1 < 250:
                    if stateMt.local_pos_1.x > 3:
                        vi = 0.08
                    print('d1 publishing set pt to decrease height to 1m')
                    pos_1.pose.position.x = stateMt.local_pos_1.x
                    pos_1.pose.position.y = stateMt.row_spawn_sp1[k][1]
                    pos_1.pose.position.z = 1.5
                    local_pos_pub_1.publish(pos_1)
                    rospy.sleep(5)
                    m += 1
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
            
                local_vel_pub_1.publish(vel_1)
                
            #if ((img_proc.position_aruco_x_0-10) < (200) < (img_proc.position_aruco_x_0+10)) and (img_proc.position_aruco_y_0 -25  < (225) < (img_proc.position_aruco_y_0)):
            if(((200 - img_proc.position_aruco_x_1)**2 + (225-img_proc.position_aruco_y_1)**2)<= (img_proc.exo_rad_1)**2) and ((225 <img_proc.position_aruco_y_1-4) and (190<img_proc.position_aruco_x_1)):
                flag1 = True
                box_id = list(img_proc.Detected_ArUco_markers_1.keys())[0]

                img_proc.box_setpoint = [
                    stateMt.local_pos_1.x, stateMt.local_pos_1.y]
                print('d1 Box is at ', img_proc.box_setpoint)
                
                pos_1.pose.position.x = img_proc.box_setpoint[0]
                pos_1.pose.position.y = img_proc.box_setpoint[1]
                pos_1.pose.position.z = 1
                local_pos_pub_1.publish(pos_1)
                ofb_ctl.setAutoLandMode_1()
                print('d1 Attempted to land c=', str(land_count))

                while not stateMt.check_gripper_1 == 'True':
                     if stateMt.local_pos_1.z < 0.25:
                        ofb_ctl.gripper_activate_1(True)
                        img_proc.aruco_thresh_bool = False
                stateMt.boxes_in_row -= 1    
                if stateMt.check_gripper_1 == 'True':
                    print('d1 The box has been gripped')
                    land_count += 1
                    box_dropped = True
                else:
                    print('d1 The box cannot yet be gripped')

                

                offboard_1()
        
                setpoint = (stateMt.local_pos_1.x,
                            stateMt.local_pos_1.y, 4)
                setpoints_1.insert(i+1, setpoint)
                i = i+1
                print('d1 i increased to ', i, 'after re-arming')
                print('d1 Setting flag1 to false again')
                
                local_pos_pub_1.publish(pos_1)

                if stateMt.check_gripper_1 == 'True':
                    truck_pts = stateMt.calculate_truck_point(box_id, 1)
                    setpoints_1.extend(
                        [truck_pts[0], truck_pts[1],(0, 0, 0)])
                    print('d1 Setpoints list as of now', setpoints_1)
                    flag1 = False

            #previous_x_error = img_proc.position_aruco_x_1 - 200
            previous_y_error = img_proc.position_aruco_y_1 - \
                (200 + 80/stateMt.local_pos_1.z)

            last_pixel_aruco_detected = img_proc.position_aruco_x_0 

        elif img_proc.aruco_thresh_bool == False:
            ofb_ctl.offboard_set_mode_1()

            pos_1.pose.position.x = setpoints_1[i][0]
            pos_1.pose.position.y = setpoints_1[i][1]
            pos_1.pose.position.z = setpoints_1[i][2]

            if reached == True and (setpoints_1[i][1] != 0 and abs(setpoints_1[i][1]) % 4 == 0):
  
                vel_1.twist.linear.x = 1.5
                vel_1.twist.linear.y = (setpoints_1[i][1] - stateMt.local_pos_1.y)/10
                vel_1.twist.linear.z = 3 - stateMt.local_pos_1.z

                flag_flip_pos_vol = True  # have to turn it false in aruco detected

            if flag_flip_pos_vol == True:
                print('d1 publishing row start velocity')
                local_vel_pub_1.publish(vel_1)
                box_dropped = False

            else:
    
                local_pos_pub_1.publish(pos_1)

            if reached == True and flag1 == False:
                print("d1 Reached goal")
                while x == 0:
                    print(k)
                    print(stateMt.row_spawn_sp1)
                    print(stateMt.row_spawn_sp1[k])
                    setpoints_1.extend([stateMt.row_spawn_sp1[k],(0,4,4)])
                    print('after reaching goal setpoints are', setpoints_1)
                    x = x+1

                i = i+1
                print('d1 i increased to ', i, 'after reaching goal')

            if i > 3 and i == (len(setpoints_1) - 2):
                ofb_ctl.setAutoLandMode_1()
                while not stateMt.local_pos_1.z<2.3:
                    lol=0              
                for o in range(3):    
                    ofb_ctl.gripper_activate_1(False)
                    
                box_dropped = True
                print("d1 Releasing box")

            elif img_proc.aruco_thresh_bool == True and len(img_proc.Detected_ArUco_markers_1) == 0:
                if last_pixel_aruco_detected<200:
                    x = -1
                else: x = 1
                
                vel_1.twist.linear.x = x
                vel_1.twist.linear.y = 0
                vel_1.twist.linear.z = 0

                local_vel_pub_1.publish(vel_1)

        rate.sleep()


if __name__ == '__main__':

    try:
        p1 = Process(target=drone_0)
        p1.start()
        p2 = Process(target=drone_1)
        p2.start()

        p1.join()
        p2.join()
    except rospy.ROSInterruptException:
        pass
