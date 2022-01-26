#! /usr/bin/env python

#import packages
import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('grape_topological_navigation')

    resume_navigation = False


    # function to pause and resume navigation whilst counting
    def control_navigation(data):
        if 'resume navigation' in str(data):
            global resume_navigation 
            resume_navigation = True


    # subscribe to commands from the control node
    sub = rospy.Subscriber('navigation_resume', String, control_navigation)
    # publish feedback to the control node
    pub = rospy.Publisher('navigation_control', String, queue_size=10)

    # create an action client to send topological navigation goals
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        # list of goals for the robot to navigate to
        goals = ["WayPoint1", "WayPoint2", "WayPoint3", "WayPoint4", "WayPoint5", "WayPoint6", "WayPoint7", "WayPoint8", "WayPoint9", "WayPointX", "WayPoint0"]
        
        for waypoint in goals:
            
            # wait for a command from the control node to resume navigation once counting is complete
            while not resume_navigation:
                rospy.Rate(1).sleep()

            rospy.loginfo("navigation resumed")
            # go to each WayPoint in the list
            goal = GotoNodeGoal()
            goal.target = waypoint
            client.send_goal(goal)
            # wait until the action is complete
            status = client.wait_for_result() 
            result = client.get_result()
            rospy.loginfo("status is %s", status)
            rospy.loginfo("result is %s", result)
            # tell the control node that the goal has been reached
            pub.publish(str(goal.target) + str(',') + str(result))
            # pause navigation until the control node publishes a command to resume navigation
            resume_navigation = False
            rospy.loginfo("navigation paused")
