#! /usr/bin/env python
# coding=utf-8


from data.srv import Goods
from data.msg import Photo

from data.srv import PhotoSrv
from data.msg import DetectionAction
from data.msg import DetectionActionGoal
from data.msg import DetectionResult
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
        self.photo_number_client = rospy.ServiceProxy("shop/photo_number_srv", PhotoSrv)
        self.action_client = actionlib.SimpleActionServer(
            'shop/detection', DetectionAction, self.execute, False)
        self.number = 0
        self.photo_number = 0
        self.flag = False
        rospy.Subscriber('/shop/photo_number',Photo,self.callback)
        self.action_client.start()
        rospy.loginfo("detection server is run")

    def execute(self, goal):
        # rospy.loginfo("detection callback")
        action_result = DetectionResult()
        if self.flag == True:
            if self.number != self.photo_number:
                self.number = self.photo_number
                result = self.detection.run_image(self.number)
                rospy.loginfo(result)
                # 为了演示的稳定性，把这些全部关了
                # if self.number == 1:
                #     self.goods_client.call(
                #         0, result['scores'][0], result['classes'][0])
                #     self.goods_client.call(
                #         1, result['scores'][1], result['classes'][1])
                #     self.goods_client.call(
                #         2, result['scores'][2], result['classes'][2])
                # elif self.number == 2:
                #     self.goods_client.call(
                #         3, result['scores'][0], result['classes'][0])
                #     self.goods_client.call(
                #         4, result['scores'][1], result['classes'][1])
                #     self.goods_client.call(
                #         5, result['scores'][2], result['classes'][2])
                # elif self.number == 3:
                #     self.goods_client.call(
                #         6, result['scores'][0], result['classes'][0])
                #     self.goods_client.call(
                #         7, result['scores'][1], result['classes'][1])
                #     self.goods_client.call(
                #         8, result['scores'][2], result['classes'][2])
                # elif self.number == 4:
                #     self.goods_client.call(
                #         9, result['scores'][0], result['classes'][0])
                #     self.goods_client.call(
                #         10, result['scores'][1], result['classes'][1])
                #     self.goods_client.call(
                #         11, result['scores'][2], result['classes'][2])
                # else:
                #     rospy.logerr("REEOR:image_num is%d",self.number)
                self.photo_number_client.call(self.number,False)
                if self.number == 4:
                    action_result.success_flag = True
                    self.action_client.set_succeeded(action_result)
                else:
                    action_result.success_flag = False
                    self.action_client.set_succeeded(action_result)               
            else:
                # rospy.logwarn("WARN:image_num%d is by distinguish",self.number)
                self.action_client.set_preempted()
        else:
            # rospy.logerr("REEOR:flag is lock")            
            self.action_client.set_preempted()

    def callback(self,Photo):
        self.photo_number = Photo.photo
        self.flag = Photo.is_discern
        # rospy.loginfo("is %d",Photo.photo)

if __name__ == '__main__':
    name = "detection_node"
    rospy.init_node(name)
    rospy.wait_for_service("shop/goods_write_srv")
    detection = DetectionNode()
    rospy.spin()
