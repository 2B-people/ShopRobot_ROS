#!/usr/bin/env python
# coding=utf-8

import os
import time
import sys
import rospy

from data.srv import ShelfBarrier


if __name__ == "__main__":
    name = "barrier_client_test"
    rospy.init_node(name)
    A_shelf_client = rospy.ServiceProxy("shop/A_shelf_barrier_read", ShelfBarrier)
    B_shelf_client = rospy.ServiceProxy("shop/B_shelf_barrier_read", ShelfBarrier)
    C_shelf_client = rospy.ServiceProxy("shop/C_shelf_barrier_read", ShelfBarrier)
    D_shelf_client = rospy.ServiceProxy("shop/D_shelf_barrier_read", ShelfBarrier)
    rospy.wait_for_service("shop/A_shelf_barrier_read")
    # call(位置，置空)
    resp = A_shelf_client.call(0,True)

    # if resp.success_flag:
    #     rospy.loginfo("0")

    for i in range(12):
        if resp.shelf_barrier_all[i]:
            rospy.loginfo("1")
        else:
            rospy.loginfo("0")


