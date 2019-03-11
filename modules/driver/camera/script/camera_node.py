#! /usr/bin/env python
#coding=utf-8

import rospy
import actionlib

import roslib
roslib.load_manifest('data')

from data.msg import CameraAction
from data.msg import CameraActionGoal

from open_camera import Camera



class CameraNode:
    def __init__(self,url,writ_time):
        self.camera = Camera(url,writ_time) 
        self.server = actionlib.SimpleActionServer("camera_action_server",CameraAction,self.execute,False)
        self.server.start()
        rospy.loginfo("camera server is run")

    def execute(self, goal):
        self.camera.get_photo(goal.number)
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("camera_node")
    
    address = rospy.get_param("camera_node/url/address",default="192.168.31.15")
    url = "http://admin:admin@"+address+":8081/"

    writ_time = rospy.get_param("camera_node/camera/wirt_time",default=20)
    server = CameraNode(url,writ_time)
    rospy.spin()
