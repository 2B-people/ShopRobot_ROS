#! /usr/bin/env python
# coding=utf-8


from data.srv import Goods
from data.msg import DetectionAction
from data.msg import DetectionActionGoal
import os
import time
import sys
import rospy
import actionlib

import roslib
roslib.load_manifest('data')

path = os.path.realpath(__file__)
path = path[:-28] + '/shopping_detection'
sys.path.append(path)

import main_detection

class DetectionNode(object):
    def __init__(self):
        self.detection = main_detection.Object_D()
        self.goods_client = rospy.ServiceProxy("shop/goods_write_srv", Goods)
        self.action_client = actionlib.SimpleActionServer(
            'detection_action_server', DetectionAction, self.execute, False)
        self.action_client.start()
        rospy.loginfo("detection server is run")

    def execute(self, goal):
        result = self.detection.run_image(goal.image_num)
        rospy.loginfo(result)
        if goal.image_num == 1:
            self.goods_client.call(
                0, result['scores'][0], result['classes'][0])
            self.goods_client.call(
                1, result['scores'][1], result['classes'][1])
            self.goods_client.call(
                2, result['scores'][2], result['classes'][2])
        elif goal.image_num == 2:
            self.goods_client.call(
                3, result['scores'][0], result['classes'][0])
            self.goods_client.call(
                4, result['scores'][1], result['classes'][1])
            self.goods_client.call(
                5, result['scores'][2], result['classes'][2])
        elif goal.image_num == 3:
            self.goods_client.call(
                6, result['scores'][0], result['classes'][0])
            self.goods_client.call(
                7, result['scores'][1], result['classes'][1])
            self.goods_client.call(
                8, result['scores'][2], result['classes'][2])
        elif goal.image_num == 4:
            self.goods_client.call(
                9, result['scores'][0], result['classes'][0])
            self.goods_client.call(
                10, result['scores'][1], result['classes'][1])
            self.goods_client.call(
                11, result['scores'][2], result['classes'][2])
        self.action_client.set_succeeded()


if __name__ == '__main__':
    name = "detection_node"
    rospy.init_node(name)
    rospy.wait_for_service("shop/goods_write_srv")
    detection = DetectionNode()
    rospy.spin()
