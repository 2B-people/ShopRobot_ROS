#!/usr/bin/env python
# coding=utf-8

import os
import time
import sys
import rospy

from data.srv import ShelfBarrier
form data.srv import Roadblock


if __name__ == "__main__":
    name = "debug_barrier_client"
    rospy.init_node(name)
    # A_shelf_client = rospy.ServiceProxy("A_shelf_barrier_wirte", ShelfBarrier)
    # B_shelf_client = rospy.ServiceProxy("B_shelf_barrier_wirte", ShelfBarrier)
    # C_shelf_client = rospy.ServiceProxy("C_shelf_barrier_wirte", ShelfBarrier)
    # D_shelf_client = rospy.ServiceProxy("D_shelf_barrier_wirte", ShelfBarrier)
    roadblock_client = rospy.ServiceProxy("shop_roadblock", Roadblock)

    roadblock_client.call(2)


    