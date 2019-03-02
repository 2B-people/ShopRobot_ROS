#! /usr/bin/env python
#coding=utf-8

import rospy
import actionlib

import roslib
roslib.load_manifest('data')
import sys
import time
import os
path = os.path.realpath(__file__)
path = path[:-28] + '/shopping_detection'

# sys.path.append('~/ShopRobot_ROS/modules/Planning/distinguish_learn/shopping_detection')
sys.path.append(path)
import main_detection
# import main_detection
# from data.msg import 
# from data.msg import 

class DetectionNode(object):
    def __init__(self):
        self.detection = main_detection.Object_D()


if __name__ == '__main__':
    rospy.init_node("detection_node")
    a = main_detection.Object_D()
    # image1 = Image.open('test_images/image1.jpg')
    box = a.run_image(1)
    print box
    rospy.spin()
