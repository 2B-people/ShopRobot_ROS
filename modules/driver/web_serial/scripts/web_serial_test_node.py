#!/usr/bin/env	python
#coding=utf-8

import time
import socket
import rospy
from std_msgs.msg import String

HOST='127.0.0.1'
PORT=8008
Buffer = 2048

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.bind((HOST,PORT))
sock.listen(5)

rospy.init_node("web_serial_test",anonymous=0)
pub=rospy.Publisher('web_pub',String,queue_size=10)

print "i am listening"

while not rospy.is_shutdown():
    con,addr=sock.accept()
    try:
       con.settimeout(5)
       buf=con.recv(Buffer)
       pub.publish(buf)
       time.sleep(1) 
    except socket.timeout:
        print "time out"
    con.send("yes i recv")

con.close()