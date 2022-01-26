#! /usr/bin/env python

#import packages
import rospy
from std_msgs.msg import String, Int32


class grape_control:
    

    def __init__(self):

        # subscribe to get information about topological navigation 
        self.nav_sub = rospy.Subscriber('navigation_control', String, self.nav_callback)
        # subscribe to get status of counting to continue navigation after reset
        self.counting_status_sub = rospy.Subscriber('counting_status', String, self.resume_nav_callback)
        # subscribe to get the number of grapes
        self.grape_sub =  rospy.Subscriber('grape_count', Int32, self.count_grapes)
        # subscribe to get input from user about localisation
        self.localise_sub = rospy.Subscriber('localisation_complete', String, self.resume_nav_callback)

        # publish commands to contol the grape counting process
        self.control_pub = rospy.Publisher('counting_control', String, queue_size=10)
        # publish commands to the navigation goal setter to resume navigation
        self.nav_pub = rospy.Publisher('navigation_resume', String, queue_size=10)

        # initialise grape counting
        self.results = []
        self.average_count = 0


    # function to pause the counting process and resume navigation once localisation or grape counting is complete
    def resume_nav_callback(self, data):
        if 'counting complete' in str(data) or 'localisation complete' in str(data):
            self.control_pub.publish('pause count')
            self.nav_pub.publish('resume navigation')
            print 'pause counting grapes, resume navigation'


    # function to start and stop counting depending on the location of the robot
    def nav_callback(self, data):
        # data is in the form (data: "WayPoint1,success: True")
        data_string = str(data).split('"')
        # get the current waypoint from the data string
        self.current_waypoint = data_string[1].split(',')[0]
        print 'Current waypoint: ', self.current_waypoint

        # define the waypoints for counting to take place at
        active_waypoints = ['WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4', 'WayPoint6', 'WayPoint7', 'WayPoint8', 'WayPoint9']

        # define the waypoints to reset the grape count at
        reset_waypoints = ['WayPoint5', 'WayPointX']
        
        # start the counting process if the robot is at one of the correct waypoints
        if self.current_waypoint in active_waypoints:
            print 'start counting grapes'
            self.control_pub.publish('start count')
        
        # reset the grape count if the robot is at one of the correct waypoints
        if self.current_waypoint in reset_waypoints:
            print 'reset count'
            self.control_pub.publish('reset count')


    # function to print the number of grapes counted and calculate the average number over both directions
    def count_grapes(self, data):
        count = int(str(data).split(': ')[1])
        # save the count in a list
        self.results.append(count)
        # calculate the average
        self.average_count =  float(sum(self.results)) / float(len(self.results))
        # display the results
        print 'Current count: ', count
        print 'List of counts: ', self.results
        print 'Average count: ', self.average_count


if __name__ == '__main__':
    rospy.init_node('grape_control')
    gc = grape_control()
    rospy.spin()
