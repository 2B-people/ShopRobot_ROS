#!/usr/bin/env python
# coding=utf-8
import os
import time
import sys
import rospy

from data.srv import Goods

if __name__ == "__main__":
    name = "debug_goods_client"
    rospy.init_node(name)
    goods_client = rospy.ServiceProxy("shop/goods_write_srv", Goods)
    # 格式(位置,分数,东西)
    goods_client.call(1,1,2)
    goods_client.call(2,2,3)
    goods_client.call(3,3,4)
    goods_client.call(4,0,2)
    goods_client.call(5,0,2)
    goods_client.call(6,0,3)
    goods_client.call(7,0,3)
    goods_client.call(8,0,3)
    goods_client.call(9,0,3)
    goods_client.call(10,0,3)
    goods_client.call(11,0,3)
