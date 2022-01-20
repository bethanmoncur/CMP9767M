#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('grape_topological_navigation')

    resume_navigation = False


    def control_navigation(data):
        if 'resume navigation' in str(data):
            global resume_navigation 
            resume_navigation = True


    pub = rospy.Publisher('navigation_control', String, queue_size=10)
    sub = rospy.Subscriber('navigation_resume', String, control_navigation)

    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

    while not resume_navigation:
        rospy.Rate(1).sleep()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint1"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))

    # send second goal
    goal.target = "WayPoint2"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))
    resume_navigation = False
    rospy.loginfo("navigation paused")

    while not resume_navigation:
        rospy.Rate(1).sleep()

    rospy.loginfo("navigation resumed")
    # send third goal
    goal.target = "WayPoint4"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))

    # send fourth goal
    goal.target = "WayPoint5"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))
    resume_navigation = False
    rospy.loginfo("navigation paused")

    while not resume_navigation:
        rospy.Rate(1).sleep()

    rospy.loginfo("navigation resumed")
    # send fifth goal
    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))


