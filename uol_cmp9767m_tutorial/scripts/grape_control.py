#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy

from std_msgs.msg import String, Int32


class grape_control:
    
    def __init__(self):
        # subscribe to get information about topological navigation 
        self.nav_sub = rospy.Subscriber('navigation_control', String, self.nav_callback)

        # publish commands to contol the grape counting process
        self.control_pub = rospy.Publisher('counting_control', String, queue_size=10)

        # subscribe to get the number of grapes
        self.grape_sub =  rospy.Subscriber('grape_count', Int32, self.count_grapes)

        # initialise grape counting
        self.results = []
        self.average_count = 0


    def nav_callback(self, data):
        # data is in the form (data: "WayPoint1,success: True")
        data_string = str(data).split('"')
        self.current_waypoint = data_string[1].split(',')[0]
        status = data_string[1].split(': ')[1]
        if status == 'True':
            self.waypoint_reached = True
        else:
            self.waypoint_reached = False
        
        print 'Current waypoint: ', self.current_waypoint
        print 'Waypoint reached: ', self.waypoint_reached
        

        if self.current_waypoint == 'WayPoint1' or self.current_waypoint == 'WayPoint4':
            print 'start counting grapes'
            self.control_pub.publish('start count')
        
        if self.current_waypoint == 'WayPoint2' or self.current_waypoint == 'WayPoint5':
            self.control_pub.publish('reset count')


    def count_grapes(self, data):
        count = int(str(data).split(': ')[1])
        self.results.append(count)
        self.average_count =  sum(self.results) / len(self.results)
        print 'Current count: ', count
        print 'List of counts: ', self.results
        print 'Average count: ', self.average_count

if __name__ == '__main__':
    rospy.init_node('grape_control', anonymous=True)
    gc = grape_control()
    rospy.spin()
