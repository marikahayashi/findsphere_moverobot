#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
from geometry_msgs.msg import Twist
import time
import serial

#g_seri = serial.Serial('/dev/ttyUSB0', 38400)

def cmd_vel_callback(cmd_vel):
    #g_seri.write('TBD')
    print cmd_vel


if __name__ == '__main__':
    g_cmd_vel_sub = rospy.Subscriber('/robot1/cmd_vel', Twist, cmd_vel_callback, queue_size=1)
    rospy.init_node('agv1_mobile_base_ctrl')
    try:
        rospy.spin()
    except KeyboardIntterupt:
        #g_seri.close()
        sys.exit()
