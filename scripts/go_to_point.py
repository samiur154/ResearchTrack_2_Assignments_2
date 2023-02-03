## @package rt2_assignment1
#	\file go_to_point.py
#	\brief This file contains the go_to_point function to describe the movement of the holonomic robot
#	\author Samuir Rehman 	
#	\date 02/02/2023
#	
#	\details
#
#	\Publisher:
#	    \cmd_vel
#
#	\Subscriber:
#	    \odom
#
#	\Action Server: <BR>
#	    \go_to_point
#
#	This node defines go_to_poin node to define movement


#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None


# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


##
# Service callback of the odometry subscriber
#
#
#    provides the X,Y,Z and angular(theta) datas from the Odom message.
#    Args:
#      msg (Odometry): odometry message.
# 


# action_server
act_s = None

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


##
#  Function to specify the state_ value
#
#   
#   Update the current global state
#    Args: state (int):  new state
#
# 

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


##
# 
# \brief Normalize the angle
# \param angle(float): float varaible for the angle to normalize
# \return: returns the normalized angle 
#
# Function to normalize the angle of the robot
###

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


###
# 
# \brief Fix the yaw angle
# \param des_pos(Point): desired position to reach a specific goal
#
# Function used to orient the robot with respect to the target position
###

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(1)


##
# 
# \brief Function to go straight
# \param des_pos(Point): the desired position to reach
#
# Function to call in case the robot has to go forward once it is correctly orriented
#
# Set the linear and angular velocity of the robot according to the distance to the goal
##
def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        change_state(2)
 
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


##
# 
# \brief Final fix to yaw angle
# \param des_yaw(float): the desired yaw
#
# Once the goal's position is acheived the function,the angular data is changed according to the goal
# 
## 

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(3)


##
#  \brief Function to call when the target is reached or when it's cancelled,
#         it sets to zero the linear and angualar velocity of the robot in order to stop the robot.
##

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


##
#    Set the appropriate behaviour depending
#    on the current robot state, in orderd
#    to reach the goal.
#    The state machine keeps running until
#    the goal is reached or when the goal is preempted 
#
#
#    Args:
#      goal (PoseActionGoal): (x,y,theta) goal pose
# 
def planning(goal):

    global state_,desired_position


    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta

    state_=0
    success = True

    feedback = rt2_assignment1.msg.PlanningFeedback()
    result = rt2_assignment1.msg.PlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            done()

            success = False
            break
        
        elif state_ == 0:
            	fix_yaw(desired_position)
        elif state_ == 1:
            	go_straight_ahead(desired_position)
        elif state_ == 2:
            	fix_final_yaw(des_yaw)
        elif state_ == 3:
            	done()
            	act_s.set_succeeded(result)
            	break
      
        else:
          rospy.logerr('Unknown state!')


#
# \brief Main function of the node
#
# The main function for this script: it initializes the publisher, the subscribers and the SimpleActionServer, in particular
#
# creates a publisher on topic /cmd_vel to publish the robot's velocity
# creates a subscriber on topic /odom to get the current position of the robot
# creates an action in order to manage the state_machine
#
##


def main():
    global pub_,active_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.PlanningAction, planning, auto_start=False)
    act_s.start()



    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
