#!/usr/bin/env python
# coding=utf-8

import os
import time
import sys
import rospy

from data.srv import ShelfBarrier
from data.srv import Roadblock


if __name__ == "__main__":
    name = "debug_barrier_client"
    rospy.init_node(name)
    A_shelf_client = rospy.ServiceProxy(
        "shop/A_shelf_barrier_wirte", ShelfBarrier)
    B_shelf_client = rospy.ServiceProxy(
        "shop/B_shelf_barrier_wirte", ShelfBarrier)
    C_shelf_client = rospy.ServiceProxy(
        "shop/C_shelf_barrier_wirte", ShelfBarrier)
    D_shelf_client = rospy.ServiceProxy(
        "shop/D_shelf_barrier_wirte", ShelfBarrier)
    roadblock_client = rospy.ServiceProxy("shop_roadblock", Roadblock)

    rospy.wait_for_service("shop/A_shelf_barrier_read")

    # roadblock_client.call(2)
    # call(位置，置空)
    A_shelf_client.call(1-1, False)
    A_shelf_client.call(2-1, True)
    A_shelf_client.call(3-1, False)
    A_shelf_client.call(4-1, True)
    A_shelf_client.call(5-1, False)
    A_shelf_client.call(6-1, True)
    A_shelf_client.call(7-1, False)
    A_shelf_client.call(8-1, False)
    A_shelf_client.call(9-1, False)
    A_shelf_client.call(10-1, True)
    A_shelf_client.call(11-1, True)
    A_shelf_client.call(12-1, False)

    B_shelf_client.call(1-1, False)
    B_shelf_client.call(2-1, True)
    B_shelf_client.call(3-1, False)
    B_shelf_client.call(4-1, False)
    B_shelf_client.call(5-1, False)
    B_shelf_client.call(6-1, True)
    B_shelf_client.call(7-1, True)
    B_shelf_client.call(8-1, False)
    B_shelf_client.call(9-1, False)
    B_shelf_client.call(10-1, False)
    B_shelf_client.call(11-1, True)
    B_shelf_client.call(12-1, True)

    C_shelf_client.call(1-1, True)
    C_shelf_client.call(2-1, False)
    C_shelf_client.call(3-1, False)
    C_shelf_client.call(4-1, True)
    C_shelf_client.call(5-1, True)
    C_shelf_client.call(6-1, False)
    C_shelf_client.call(7-1, True)
    C_shelf_client.call(8-1, True)
    C_shelf_client.call(9-1, True)
    C_shelf_client.call(10-1, False)
    C_shelf_client.call(11-1, False)
    C_shelf_client.call(12-1, True)

    D_shelf_client.call(1-1, False)
    D_shelf_client.call(2-1, False)
    D_shelf_client.call(3-1, True)
    D_shelf_client.call(4-1, True)
    D_shelf_client.call(5-1, False)
    D_shelf_client.call(6-1, False)
    D_shelf_client.call(7-1, False)
    D_shelf_client.call(8-1, True)
    D_shelf_client.call(9-1, False)
    D_shelf_client.call(10-1, False)
    D_shelf_client.call(11-1, False)
    D_shelf_client.call(12-1, False)
