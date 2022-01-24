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

        # subscribe to get status of counting to continue navigation after reset
        self.counting_status_sub = rospy.Subscriber('counting_status', String, self.resume_nav_callback)

        # publish commands to the navigation goal setter to resume navigation
        self.nav_pub = rospy.Publisher('navigation_resume', String, queue_size=10)

        # initialise grape counting
        self.results = []
        self.average_count = 0

        # subscribe to get input from user about localisation
        self.localise_sub = rospy.Subscriber('localisation_complete', String, self.resume_nav_callback)


    def resume_nav_callback(self, data):
        if 'counting complete' in str(data) or 'localisation complete' in str(data):
            self.control_pub.publish('pause count')
            rospy.sleep(1.5)
            self.nav_pub.publish('resume navigation')
            print 'pause counting grapes, resume navigation'


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

        active_waypoints = ['WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4', 'WayPoint6', 'WayPoint7', 'WayPoint8', 'WayPoint9']
        reset_waypoints = ['WayPoint5', 'WayPointX']
        

        if self.current_waypoint in active_waypoints:
            rospy.sleep(3.0)
            print 'start counting grapes'
            self.control_pub.publish('start count')
        
        if self.current_waypoint in reset_waypoints:
            print 'reset count'
            self.control_pub.publish('reset count')



    def count_grapes(self, data):
        count = int(str(data).split(': ')[1])
        self.results.append(count)
        self.average_count =  float(sum(self.results)) / float(len(self.results))
        print 'Current count: ', count
        print 'List of counts: ', self.results
        print 'Average count: ', self.average_count


if __name__ == '__main__':
    rospy.init_node('grape_control')
    gc = grape_control()
    rospy.spin()
