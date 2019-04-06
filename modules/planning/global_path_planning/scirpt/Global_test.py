#! /usr/bin/env python

import roslib
roslib.load_manifest("data")
import rospy
import actionlib

from data.msg import GlobalPlanAction
from data.msg import GlobalPlanGoal

if __name__ == '__main__':
    rospy.init_node('test')
    client = actionlib.SimpleActionClient('global_plan', GlobalPlanAction)
    client.wait_for_server()
    goal = GlobalPlanGoal()
    goal.do_flag = True
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    while True:
        pass