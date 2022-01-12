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
    pub = rospy.Publisher('navigation_control', String, queue_size=10)
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

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

    # send fourth goal
    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    pub.publish(str(goal.target) + str(',') + str(result))


