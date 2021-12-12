#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from std_msgs.msg import String


class offboard_control:

    def __init__(self):
    
        rospy.init_node('offboard_control', anonymous=True)


    
    
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
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Autoland Mode could not be set" % e)
    




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

        #callback to check if gripper can be activated or not
    
    
    def gripper_check_clbk(self, msg):
        check_gripper = msg.data 
        #rospy.loginfo(check_gripper)
        return check_gripper









def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    
    
    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    
    
    
    # Specify the rate
    rate = rospy.Rate(20.0)



    # Make the list of setpoints
    setpoints = [(0, 0, 3), (3, 0, 3), '''(3, 0, 0)''', (3, 0, 3), (3, 3, 3), '''(3, 3, 0)''', (3, 3, 3), (0, 0, 3), '''(0, 0, 0)''']  # List to setpoints


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
    rospy.Subscriber('/gripper_check', String,stateMt.gripper_check_clbk)



    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        print('Sending dummy points')
        local_pos_pub.publish(pos)
        rate.sleep()


    while not stateMt.state.armed:  #safety check later
        ofb_ctl.setArm()
        rate.sleep()
        print("Armed!!")

    
    while not stateMt.state.mode == "OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
        print("OFFBOARD mode activated")
    i = 0
    def check_position():
        desired = np.array((setpoints[i][0], setpoints[i][1], setpoints[i][2]))
        pos = np.array((stateMt.local_pos.x,
                        stateMt.local_pos.y,
                        stateMt.local_pos.z))
        print(np.linalg.norm(desired - pos))
        
        return np.linalg.norm(desired - pos) < 0.5





    while not rospy.is_shutdown():


        stateMt
        ofb_ctl.setArm
        ofb_ctl.offboard_set_mode
        reached = check_position()
        if (reached == True) and (i <= 4):
            i += 1
            print(i)
            if i==5:
               offboard_control.setAutoLandMode
            
            
            

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
