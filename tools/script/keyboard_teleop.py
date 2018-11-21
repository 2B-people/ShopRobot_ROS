#!/usr/bin/env python
import rospy
import cv2
from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

msg = """
Control The Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d

t/y : increase/decrease only linear speed by 10%
g/h : increase/decrease only angular speed by 10%
space key, space : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

speedBindings = {
    't': (0.1, 0.1, 0),
    'y': (-0.1, -0.1, 0),
    'g': (0, 0, 0.1),
    'h': (0, 0, -0.1),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


linear_x_now = 1
linear_y_now = 1
angular_z_now = 1


def vels(linear_x, linear_y, angular_z):
    return "linear_x_now:%s\tlinear_y_now%s\tangular_z_now %s " % (linear_x, linear_y, angular_z)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    rospy.set_param("LINEAR_X_MAX",3)
    rospy.set_param("LINEAR_Y_MAX",3)
    rospy.set_param("ANGULAR_Z_MAX",3)

    linear_x_max = rospy.get_param("LINEAR_X_MAX")
    linear_y_max = rospy.get_param("LINEAR_Y_MAX")
    angular_z_max = rospy.get_param("ANGULAR_Z_MAX")

    x = 0
    y = 0
    turn = 0
    conut = 0
    status = 0
    linear_x_target = 0
    linear_y_target = 0
    angular_z_target = 0
    linear_x_control = 0
    linear_y_control = 0
    angular_z_control = 0
    
    try:
        print msg       
        print vels(linear_x_now, linear_y_now, angular_z_now)

        while (1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                turn = moveBindings[key][2]
                conut = 0
            elif key in speedBindings.keys():
                linear_x_now = linear_x_now + speedBindings[key][0]
                linear_y_now = linear_y_now + speedBindings[key][1]
                angular_z_now = angular_z_now + speedBindings[key][2]
                conut = 0

                if linear_x_now > linear_x_max:
                    linear_x_now = linear_x_max
                if linear_y_now > linear_y_max:
                    linear_y_now = linear_y_max
                if angular_z_now > angular_z_max:
                    angular_z_now = angular_z_max
                
                if linear_x_now < 0:
                    linear_x_now = 0
                if linear_y_now < 0:
                    linear_y_now = 0
                if angular_z_now < 0:
                    angular_z_now = 0                
                print vels(linear_x_now, linear_y_now, angular_z_now)
                if (status == 10):
                    print msg
                status = (status + 1) % 11
            elif key == ' ' or key == 'k':
                x = 0
                y = 0
                turn = 0
            else:
                conut = conut + 1
                if conut > 4:
                    x = 0
                    y = 0
                    turn = 0
                if (key == '\x03'):
                    break

            linear_x_target = x * linear_x_now
            linear_y_target = y * linear_y_now
            angular_z_target = turn * angular_z_now

            # control x speed cmd_vel
            if linear_x_target > linear_x_control:
                linear_x_control = min(linear_x_target, linear_x_control+0.1)
            elif linear_x_target < linear_x_control:
                linear_x_control = max(linear_x_target, linear_x_control-0.1)
            else:
                linear_x_control = linear_x_target
            # control y speed cmd_vel
            if linear_y_target > linear_y_control:
                linear_y_control = min(linear_y_target, linear_y_control+0.1)
            elif linear_x_target < linear_x_control:
                linear_y_control = max(linear_y_target, linear_y_control-0.1)
            else:
                linear_y_control = linear_y_target
            # control z speed cmd_vel
            if angular_z_target > angular_z_control:
                angular_z_control = min(
                    angular_z_target, angular_z_control+0.1)
            elif angular_z_target < angular_z_control:
                angular_z_control = max(
                    angular_z_target, angular_z_control-0.1)
            else:
                angular_z_control = angular_z_target

            # topic cmd_vel
            twist = Twist()
            twist.linear.x = linear_x_control
            twist.linear.y = linear_y_control
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = angular_z_control
            pub.publish(twist)

    except:
        rospy.logerr("kill keyborad_teleop by export")
    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
