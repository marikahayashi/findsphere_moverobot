#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from findsphere_moverobot.srv import StartStop
import InputTools
import threading
import time

g_startstop = None
g_image_sub = None
g_bridge = CvBridge()
g_cvimage = None
g_lock = threading.Lock()
g_stop_event = None
g_imshow_thread = None
g_imshow_width = 320

def main():
    global g_stop_event, g_imshow_thread
    key = InputTools.waitKey(100)
    if key == 'q':
        g_stop_event.set()
        g_imshow_thread.join()
        rospy.signal_shutdown("User hit q key to quit.")
    elif key == 't':
        response = g_startstop(True)
    elif key == 'p':
        response = g_startstop(False)

    # always start
    response = g_startstop(True)
        

def image_callback(image):
    global g_bridge, g_cvimage, g_lock
    g_lock.acquire()
    g_cvimage = g_bridge.imgmsg_to_cv2(image,"bgr8")
    g_lock.release()
    #cv2.imshow('processed_image', cvimage)
    #cv2.waitKey(1)
    # ^^^ doesn't reconnect after other node restarts


def imshow_thread_func():
    global g_cvimage, g_lock, g_stop_event, g_imshow_width
    while not g_stop_event.is_set():
        time.sleep(0.05)
        if g_cvimage is None:
            continue
        g_lock.acquire()
        width = g_cvimage.shape[0]
        height = g_cvimage.shape[1]
        resized_image = cv2.resize(g_cvimage,
                                   (height * g_imshow_width / width,
                                    g_imshow_width))
        g_lock.release()
        cv2.imshow('processed_image', resized_image)
        key = cv2.waitKey(1)
        if key == 'q':
            return


if __name__ == '__main__':
    g_image_sub = rospy.Subscriber('/processed_image', Image, image_callback, queue_size=1)
    g_imshow_thread = threading.Thread(target=imshow_thread_func, name="imshow_thread")
    g_stop_event = threading.Event()
    g_imshow_thread.start()
    try:
        rospy.init_node('findsphere_monitor')
        g_startstop = rospy.ServiceProxy('startstop', StartStop)
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            main()
            rate.sleep()
    except KeyboardInterrupt:
        sys.exit()


