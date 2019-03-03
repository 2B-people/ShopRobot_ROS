#! /usr/bin/env python
#coding=utf-8

import rospy
import actionlib

import roslib
roslib.load_manifest('data')

from data.msg import CameraAction
from data.msg import CameraActionGoal

import open_camera



class CameraNode:
    def __init__(self,url,writ_time):
        self.camera = open_camera.Camera(url,writ_time) 
        self.server = actionlib.SimpleActionServer("camera/action",CameraAction,self.execute,False)
        self.server.start()
        rospy.loginfo("camera server is run")

    def execute(self, goal):
        camera.get_photo(goal.number)
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("camera_node")
    url = rospy.get_param("url/address",default="http://admin:admin@192.168.31.102:8081/")
    server = CameraNode(url,path,writ_time)
    rospy.spin()