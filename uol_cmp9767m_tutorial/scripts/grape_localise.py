#!/usr/bin/env python

#import packages
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import Empty
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from math import atan2

# create global variables to define the current position of the robot from odometry    
x = 0.0
y = 0.0
theta = 0.0


# funtion to update the position of the robot from odometry data    
def new_odom(data):
    global x
    global y
    global theta
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    

rospy.init_node('move_to_localise', anonymous=True)

# subscribe to the odometry topic
odom_sub = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, new_odom)

# publish velocity commands to the robot
vel_pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size = 1)

# publish feedback to the control node
localise_pub = rospy.Publisher('localisation_complete', String, queue_size = 1)

# create a service proxy to enable redistributing of the amcl particles
global_localization = rospy.ServiceProxy('global_localization', Empty)

# ask the user if the robot needs localising
start_input = raw_input('Does the robot need localising? [y/n] \n')

# tell the control node that the robot is localised    
if start_input == 'n':
    localise_pub.publish('localisation complete')
    quit()

# comment out to speed up initial localisation if using an accurate initial pose estimate for amcl    
#else:
    #global_localization()

# create an object to send velocity commands  
t = Twist()

# define a rate of 10hz
r = rospy.Rate(10)

# define the odometry targets to send to the robot
i = 0
goals = [[-8.0, -3.0], [0.0, 0.0], [5.0, -1.0], [-5.0, 1.0]]

while not rospy.is_shutdown():

    # set the goal location
    goal = Point()
    goal.x = goals[i][0]
    goal.y = goals[i][1]
    
    # calculate the heading change required to get to the target position 
    dif_x = goal.x - x
    dif_y = goal.y - y
    angle_to_goal = atan2(dif_y, dif_x)
    
    # move the robot towards the goal if it is further than 1m away in either the x or y direction 
    if abs(dif_x) > 1.0 or abs(dif_y) > 1.0:
    
        if (angle_to_goal - theta) > 0.1:
            t.linear.x = 0.0
            t.angular.z = 0.3
        elif (angle_to_goal - theta) < -0.1:
            t.linear.x = 0.0
            t.angular.z = -0.3
        else:
            t.linear.x = 0.8
            t.angular.z = 0.0
        # publish the velocity command
        vel_pub.publish(t)
    
    else:
        # ask the user if the robot is localised
        user_input = raw_input('Has the robot successfully localised itself? [y/n] \n')
        if user_input == 'n':
            # redistribute the amcl particles and move to the next goal
            global_localization()
            if i == len(goals) - 1:
                i = 0
            else:
                i +=1
        elif user_input == 'y':
            # feedback to the control node
            localise_pub.publish('localisation complete')
            break
            
    # complete the while loop at a rate of 10 hz    
    r.sleep()

