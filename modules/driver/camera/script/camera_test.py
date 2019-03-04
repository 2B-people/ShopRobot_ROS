#! /usr/bin/env python

import roslib
roslib.load_manifest("data")
import rospy
import actionlib

from data.msg import CameraAction
from data.msg import CameraGoal

if __name__ == '__main__':
    rospy.init_node('camera_test_client')
    client = actionlib.SimpleActionClient('camera_action_server', CameraAction)
    client.wait_for_server()
    goal = CameraGoal
    goal.number = 1
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
