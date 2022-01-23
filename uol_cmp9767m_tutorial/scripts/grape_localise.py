#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import Empty
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def new_odom(data):
    global x
    global y
    global theta
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rot_q = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
rospy.init_node('move_to_localise', anonymous=True)

odom_sub = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, new_odom)
vel_pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size = 1)
localise_pub = rospy.Publisher('localisation_complete', String, queue_size = 1)
global_localization = rospy.ServiceProxy('global_localization', Empty)


start_input = raw_input('Does the robot need localising? [y/n] \n')
    
if start_input == 'n':
    localise_pub.publish('localisation complete')
    quit()
    
else:
    global_localization()

t = Twist()

r = rospy.Rate(10)

i = 0
goals = [[0.0, 0.0], [-8.0, -3.0], [5.0, -1.0]]

while not rospy.is_shutdown():


    goal = Point()
    goal.x = goals[i][0]
    goal.y = goals[i][1]
    
    dif_x = goal.x - x
    dif_y = goal.y - y
    angle_to_goal = atan2(dif_y, dif_x)
    
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
        
        vel_pub.publish(t)
    
    else:
        user_input = raw_input('Has the robot successfully localised itself? [y/n] \n')
        if user_input == 'n':
            global_localization()
            if i == len(goals) - 1:
                i = 0
            else:
                i +=1
        elif user_input == 'y':
            localise_pub.publish('localisation complete')
            break
            
        
    r.sleep()

