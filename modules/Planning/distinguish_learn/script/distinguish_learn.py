#! /usr/bin/env python
#coding=utf-8

import rospy
import actionlib

import roslib
roslib.load_manifest('data')
import sys
import time
sys.path.append("/home/nqq09/Shop_robot_WS/src/ShopRobot_ROS/modules/Planning/distinguish_learn/shopping_detection")
import main_detection

from data.msg import 
from data.msg import 

class DetectionNode(object):
    def __init__(self):
        self.detection = main_detection.Object_D()


if __name__ == '__main__':
    rospy.init_node("detection_node")

    server = CameraNode(url,path,writ_time)
    rospy.spin()
