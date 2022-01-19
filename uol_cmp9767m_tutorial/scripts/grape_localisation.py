#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty



def move():
        move_pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_to_localise', anonymous=True)
        global_localization = rospy.ServiceProxy('global_localization', Empty)
        t = Twist()
        iterations = 0
        rate = rospy.Rate(1)
        rospy.sleep(1.0)
        start = rospy.Time.now()
        global_localization()

        
        while not rospy.is_shutdown():

            if (rospy.Time.now() - start) < rospy.Duration(secs=7):
                # move forward
                print 'moving forward'
                t.linear.x = 2.5
                t.angular.z = 0.0
                move_pub.publish(t)
                print 'command published'
              
                
            elif (rospy.Time.now() - start) < rospy.Duration(secs=13):
                # rotate 
                print 'rotating'
                t.linear.x = 0.0
                t.angular.z = 1.6
                move_pub.publish(t)
                print 'command published'
                
            elif (rospy.Time.now() - start) < rospy.Duration(secs=19):
                # rotate back
                print 'rotating'
                t.linear.x = 0.0
                t.angular.z = -1.6
                move_pub.publish(t)
                print 'command published'
                
            elif (rospy.Time.now() - start) < rospy.Duration(secs=25):
                # move forward
                print 'moving forward'
                t.linear.x = -2.5
                t.angular.z = 0.0
                move_pub.publish(t)
                print 'command published'
                
            else:
                iterations += 1
                user_input = raw_input('Has the robot successfully localised itself? [y/n] \n')
                if user_input == 'n':
                    if iterations // 2 > 0:
                        global_localization()
                        iterations = 0
                        
                    start = rospy.Time.now()
                else:
                    return    
            
            rate.sleep()
            

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass

