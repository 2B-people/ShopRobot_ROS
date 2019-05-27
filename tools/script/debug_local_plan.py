#!/usr/bin/env python
# coding=utf-8
import os
import time
import sys
import rospy

from data.srv import Goods
from data.msg import Coord

if __name__ == "__main__":
    name = "debug_goods_client"
    rospy.init_node(name)
    goods_client = rospy.ServiceProxy("shop/goods_write_srv", Goods)
    # pub = rospy.Publisher('robot1_web/coord_now', Coord, queue_size=10)
    # 格式(位置,分数,东西)
    goods_client.call(0, 1, 1)
    goods_client.call(1, 1, 2)
    goods_client.call(2, 2, 3)
    goods_client.call(3, 3, 4)
    goods_client.call(4, 0, 5)
    goods_client.call(5, 0, 6)
    goods_client.call(6, 0, 7)
    goods_client.call(7, 0, 8)
    goods_client.call(8, 0, 9)
    goods_client.call(9, 0, 10)
    goods_client.call(10, 0,11)
    goods_client.call(11, 0,12)

    # goods_client.call(0, 1, 1)
    # goods_client.call(1, 1, 2)
    # goods_client.call(2, 2, 3)
    # goods_client.call(3, 3, 4)
    # goods_client.call(4, 0, 5)
    # goods_client.call(5, 0, 6)
    # goods_client.call(6, 0, 7)
    # goods_client.call(7, 0, 8)
    # goods_client.call(8, 0, 9)
    # goods_client.call(9, 0, 10)
    # goods_client.call(10, 0, 11)
    # goods_client.call(11, 0, 12)
    rate = rospy.Rate(1)
    x = 1
    y = 2
    pose = 1
    # while not rospy.is_shutdown():
    #     pub.publish(Coord(x,y,pose))
    #     rate.sleep()