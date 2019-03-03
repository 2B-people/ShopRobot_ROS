#! /usr/bin/env python

import roslib
roslib.load_manifest("data")
import rospy
import actionlib

from data.msg import DetectionAction
from data.msg import DetectionGoal

if __name__ == '__main__':
    rospy.init_node('distinguish_test_client')
    client = actionlib.SimpleActionClient('detection_action_server', DetectionAction)
    client.wait_for_server()
    goal = DetectionGoal()
    goal.image_num = 1
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))