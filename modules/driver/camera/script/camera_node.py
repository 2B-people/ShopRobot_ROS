#! /usr/bin/env python
#coding=utf-8

import rospy
import actionlib

import roslib
roslib.load_manifest('data')

from data.msg import CameraAction
from data.msg import CameraActionGoal
from data.srv import PhotoSrv
from data.msg import Photo

from open_camera import Camera

class CameraNode:
    def __init__(self,url,writ_time):
        self.camera = Camera(url,writ_time)
        # self.camera = Camera(1,writ_time)         
        self.server = actionlib.SimpleActionServer("shop/camera",CameraAction,self.execute,False)
        self.photo_number_client = rospy.ServiceProxy("shop/photo_number_srv", PhotoSrv)
        self.number = 0
        rospy.wait_for_service("shop/photo_number_srv")
        self.server.start()
        rospy.loginfo("camera server is run")

    def execute(self, goal):
        rospy.loginfo("camera server is execute")
        self.number = self.number + 1 
        self.camera.get_photo(self.number)
        a=self.photo_number_client.call(self.number,True)
        rospy.loginfo("camera is photo %d",self.number)
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("camera_node")
    
    address = rospy.get_param("camera_node/url/address",default="192.168.31.195")
    url = "http://admin:admin@"+address+":8081/"

    writ_time = rospy.get_param("camera_node/camera/wirt_time",default=20)
    server = CameraNode(url,writ_time)
    rospy.spin()