#! /usr/bin/env python

import roslib
roslib.load_manifest("data")
import rospy
import actionlib

from data.msg import LocalPlanAction
from data.msg import LocalPlanGoal

from data.srv import ActionName

if __name__ == '__main__':
    rospy.init_node('test')
    client = actionlib.SimpleActionClient('shop/local_plan', LocalPlanAction)
    client.wait_for_server()
    goal = LocalPlanGoal()

    client.send_goal(goal)

    client.wait_for_result(rospy.Duration.from_sec(10.0))
    while True:
        pass