#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_srvs.srv import Empty

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
global_localization = rospy.ServiceProxy('global_localization', Empty)
global_localization()

t = Twist()

r = rospy.Rate(4)

i = 0
goals = [[5.0, -3.0], [-5.0, -3.0]]

while not rospy.is_shutdown():

    goal = Point()
    goal.x = goals[i][0]
    goal.y = goals[i][1]
    
    dif_x = goal.x - x
    dif_y = goal.y - y
    angle_to_goal = atan2(dif_y, dif_x)
    
    if abs(dif_x) > 0.2 or abs(dif_y) > 0.2:
    
        if abs(angle_to_goal - theta) > 0.1:
            t.linear.x = 0.0
            t.angular.z = 0.3
        else:
            t.linear.x = 0.5
            t.angular.z = 0.0
        
        vel_pub.publish(t)
    
    else:
        if i < (len(goals) - 1):
            i +=1
            
        user_input = raw_input('Has the robot successfully localised itself? [y/n] \n')
        if user_input == 'n':
            global_localization()
        
    r.sleep()

