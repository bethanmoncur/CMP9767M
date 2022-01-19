#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist



def move():
        move_pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_to_localise', anonymous=True)
        t = Twist()
        rate = rospy.Rate(1)
        rospy.sleep(1.0)
        start = rospy.Time.now()

        
        while not rospy.is_shutdown():

            if (rospy.Time.now() - start) < rospy.Duration(secs=6):
                # move forward
                print 'moving forward'
                t.linear.x = 1.0
                t.angular.z = 0.0
                move_pub.publish(t)
                print 'command published'
              
                
            elif (rospy.Time.now() - start) < rospy.Duration(secs=19):
                # rotate 360 degrees
                print 'rotating'
                t.linear.x = 0.0
                t.angular.z = 1.0
                move_pub.publish(t)
                print 'command published'
                
            elif (rospy.Time.now() - start) < rospy.Duration(secs=25):
                # rotate 360 degrees
                print 'rotating'
                t.linear.x = -1.0
                t.angular.z = 0.0
                move_pub.publish(t)
                print 'command published'
                
            else:
                print 'movement complete'
            
            rate.sleep()
            

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass

