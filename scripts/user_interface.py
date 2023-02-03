## @package rt2_assignment1
#	\file user_interface.py
#	\brief This file contain the user interface to start and stop the robot
#	\author Samiur Rahman	
#	\date 02/02/2023
#	
#	\details
#
#	\Client : <BR>
#		\user_interface
#
#	This node define the command line user interface for the control of the robot

import rospy
import time
from rt2_assignment1.srv import Command


## 
# \breif The main fuction of the script
#
# It initializes the client to /user_interface.
# if the user presses 1 sends a "start" message, otherwise it sends a "stop" message
#
##
def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
