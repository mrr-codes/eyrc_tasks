#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import Gripper
import time


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
            change_mode("COM_RCL_EXCEPT", 2)
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


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [(0, 0, 3), (3, 0, 3), (3, 0, 3), (3, 3, 3),
                 (3, 3, 3), (0, 0, 3)]  # List to setpoints

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

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    def dummy_points():
        for i in range(100):
            print('Sending dummy points')
            local_pos_pub.publish(pos)
            rate.sleep()
    dummy_points()
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
        print(np.linalg.norm(desired - pos))

        return np.linalg.norm(desired - pos) < 0.1

    # Publish the setpoints

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
        if reached == True and (i == 1 or i == 4 or i == 5):

            ofb_ctl.setAutoLandMode()
            rospy.sleep(10)  # trying to create time for grip
            print('Attempted to land')
            # print(reached)
            if (i <= 5):
                if (i<5):
                    i = i + 1
                if stateMt.check_gripper == 'True':
                        ofb_ctl.gripper_activate(True)
                        print("gripper_true_inside_if")
                elif i == 5:
                    # stateMt.gripper_check_clbk()
                    ofb_ctl.gripper_activate(False)
                

        if reached == True & i<5:
            print("off", i)
            i = i+1

            print(i)
            dummy_points()
            ofb_ctl.offboard_set_mode()
        print(setpoints[i])

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
