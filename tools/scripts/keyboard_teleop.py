#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control The Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
        m

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == ""__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    linear_x_max = rospy.get_param("LINEAR_X_MAX")	
    linear_y_max = rospy.get_param("LINEAR_Y_MAX")	
    angular_z_max = rospy.get_param("ANGULAR_Z_MAX")



