#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from findsphere_moverobot.srv import StartStop
from findsphere_moverobot.srv import StartStopResponse
from geometry_msgs.msg import Twist
import time
import gc
import copy



class FindSphere():
    STATIC_IMAGE = 1
    VIDEO = 2
    CAMERA = 3
    
    def __init__(self, image_type):
        self.image_type = image_type
        self.node_name = "findsphere_pubvel"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)
        self.startstop_srv = rospy.Service('startstop',
                                           StartStop,
                                           self.startstop_srv_handler)
        self.move_flag = False
        self.arrival_flag = False
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        #self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        #self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        if self.image_type == FindSphere.CAMERA:
            self.capture = cv2.VideoCapture(0)
            #self.orig_img_width = 1280
            #self.orig_img_height = 720
            self.orig_img_width = 320
            self.orig_img_height = 240
            self.capture.set(3, self.orig_img_width)
            self.capture.set(4, self.orig_img_height)
            self.resized_img_width = 160 # must be exactly divisible
            #self.resized_img_height = 90 # must be exactly divisible
            self.resized_img_height = 120 # must be exactly divisible
        elif self.image_type == FindSphere.STATIC_IMAGE:
            self.orig_img_width = 320
            self.orig_img_height = 240
            self.resized_img_width = 320 # must be exactly divisible
            self.resized_img_height = 240 # must be exactly divisible
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101046230000.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101046460011.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101046480012.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101046520014.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047020019.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047050020.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047090022.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047130024.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047170026.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047260030.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047300032.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047320033.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047340034.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047360035.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047400037.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047430038.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047450039.jpg'
#            self.input_image_file_name = '/home/upr/Pictures/findsphere_images/20170410/live201704101047570045.jpg'
            self.input_image_file_name = '/home/qtmember/Pictures/20170425-2/005.jpg'
        elif self.image_type == FindSphere.VIDEO:
            #self.capture = cv2.VideoCapture('/home/upr/Pictures/findsphere_images/20170410vga/20170410vga.avi')
            self.capture = cv2.VideoCapture('/home/qtmember/Pictures/20170425-2/20170425-2.avi')
            self.orig_img_width = 320
            self.orig_img_height = 240
            #self.capture.set(3, self.orig_img_width)
            #self.capture.set(4, self.orig_img_height)
            self.resized_img_width = 320 # must be exactly divisible
            self.resized_img_height = 240 # must be exactly divisible
            

        self.yvel_coef = 0.2
        self.xvel_coef = 0.3
        self.blur_radius = 10
        self.binimg_thre = 230
        #self.shape_thre = 1
        self.shape_coef = 20.0
        self.max_blob_num = 1000
        self.ignore_area_lt = 4
        self.ignore_area_mt = 130
        self.expanded_rectangle_size = 0
        #self.expanded_corner_thre = 150*4
        self.expanded_corner_coef = 1
        self.prev_centroid_area = None
        self.prev_centroid_coef = 0.5
        self.eval_func_thre = 700
        self.center_line_thre = 40
        self.time_print_enum = 0
        self.prev_time = time.time()
        self.prev_biggest_idx = None
        self.keep_move_forward_counter = 0
        self.keep_move_forward_init = 10
        self.keep_move_forward_finished = True
        self.prev_xvel = 0
        self.prev_vel = Twist()
        

    def print_difftime(self):
        self.time_print_enum += 1
        curtime = time.time()
        #print '[%d]time=%f' % (self.time_print_enum, (curtime - self.prev_time))
        self.prev_time = curtime

        
    def capture_and_process(self):
        if self.image_type == FindSphere.VIDEO:
            time.sleep(1)
        if (self.image_type == FindSphere.CAMERA) or \
           (self.image_type == FindSphere.VIDEO):
            ret, frame = self.capture.read()
        else:  # STATIC_IMAGE
            frame = cv2.imread(self.input_image_file_name)
        frame = np.array(frame, dtype=np.uint8)

        processed_image, blob_centroid_area,\
            biggest_idx, too_big_idxs = self.find_biggest_blob(frame)
        self.prev_centroid_area = blob_centroid_area

        self.publish_processed_image(processed_image)

        #print "move_flag="
        #print self.move_flag
        self.publish_vel(blob_centroid_area, biggest_idx, too_big_idxs)

        if self.arrival_flag is True:
            self.prev_centroid_area = None

        # cv2.imshow(self.node_name, processed_image)
        # keystroke = cv2.waitKey(1)
        # if keystroke != -1:
        #     cc = chr(keystroke & 255).lower()
        #     if cc == 'q':
        #         rospy.signal_shutdown("User hit q key to quit.")
        del processed_image
        gc.collect()
        

    def startstop_srv_handler(self, req):
        self.move_flag = req.startstop
        self.arrival_flag = False
        if req.startstop is True:
            rospy.loginfo('received start command')
        else:
            rospy.loginfo('received stop command')
        return StartStopResponse(result=True)

    
    def find_biggest_blob_dummy(seld, frame):
        blob_centroid_area = [50, 50, 100]
        processed_image = frame
        biggest_idx = 1
        too_big_idx = 2
        return processed_image, blob_centroid_area, biggest_idx, too_big_idx
    
    
    def find_biggest_blob(self, frame):
        self.time_print_enum = 0
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray_img = cv2.blur(gray_img, (self.blur_radius,self.blur_radius))

        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
        #h = hsv[:, :, 0]
        #s = hsv[:, :, 1]
        #mask = np.zeros(h.shape, dtype=np.uint8)

        mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        for x in range(frame.shape[0]):
            for y in range(frame.shape[1]):
                if (frame[x][y][2] > 200) and \
                   (float(frame[x][y][0]) / frame[x][y][2] < 0.78) and \
                   (float(frame[x][y][1]) / frame[x][y][2] < 0.78):
                    mask[x][y] = 255
                else:
                    mask[x][y] = 0


        # print mask.shape

        #mask[((h < 20) | (h > 200)) & (s > 220)] = 255
        
        #cv2.imshow('gray', gray_img)
        #cv2.waitKey(1)

        #ret, binimg = cv2.threshold(gray_img, self.binimg_thre, 255, cv2.THRESH_BINARY)


        
        #blur_frame = cv2.blur(frame, (self.blur_radius, self.blur_radius))
        #hsv = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)
        #lower_red1 = np.array([160, 10, 80])
        #lower_red1 = np.array([160, 200, 200])
        #upper_red1 = np.array([190, 255, 255])
        #mask = cv2.inRange(hsv, lower_red1, upper_red1)




        
        binimg = mask
        
        self.print_difftime()
        #num_labels, label_image = cv2.connectedComponents(binimg)
        label = cv2.connectedComponentsWithStats(binimg)
        num_labels = label[0] # including background
        label_image = label[1]
        blob_centroids = np.delete(label[3], 0, 0)  #not including background
        blob_dims = label[2]  # including background

        
        
        
        
        #blob_idxs = list(set(blob_idxs)) # delete duplicate
        
        #find biggest blob among blobs
        #which are similar to circle
        biggest_idx = None
        prev_bdsize = 0
        too_big_idxs = []
        mid_size_blob_idxs = []
        blob_idxs = []
        expanded_rects_corners = []
        eval_func = []  # target scores lower value
        for i in range(num_labels-1):
            eval_func.append(-1)
            # ignore too big blob
            if blob_dims[i + 1][4] > self.ignore_area_mt:
                too_big_idxs.append(i)
                continue
            # ignore too small blob
            if blob_dims[i + 1][4] < self.ignore_area_lt:
                continue
            # ignore left/right side blobs
            if (blob_dims[i + 1][0] < 15) or \
               (blob_dims[i + 1][2] + blob_dims[i+1][0] \
                > self.orig_img_width - 15):
                continue

            mid_size_blob_idxs.append(i)

            
            # calc shape evaluation value
            blob_upper_height = blob_centroids[i][1] - blob_dims[i + 1][1]
            blob_left_width = blob_centroids[i][0] - blob_dims[i + 1][0]
            expected_radius = np.sqrt(blob_dims[i + 1][4] / np.pi)
            # add each evaluation value (+=) , not multiply (*=)
            eval_func[i] = (abs(blob_upper_height - expected_radius) \
                             + abs(blob_left_width - expected_radius)) \
                             * self.shape_coef  # min = 0

            # calc eval value of expanded bounding box's corner's darkness
            # if dark, then assume target,
            # if not dark, then assume smooth plane i.e. floor
            top_left_x = blob_dims[i + 1][0] - self.expanded_rectangle_size
            if top_left_x < 0:
                top_left_x = 0
            top_left_y = blob_dims[i + 1][1] - self.expanded_rectangle_size
            if top_left_y < 0:
                top_left_y = 0
            bottom_right_x = blob_dims[i + 1][0] + blob_dims[i + 1][2] \
                             + self.expanded_rectangle_size
            if bottom_right_x >= self.orig_img_width:
                bottom_right_x = self.orig_img_width -1
            bottom_right_y = blob_dims[i + 1][1] + blob_dims[i + 1][3] \
                             + self.expanded_rectangle_size
            if bottom_right_y >= self.orig_img_height:
                bottom_right_y = self.orig_img_height -1
            expanded_rects_corners.append(
                [(top_left_x, top_left_y),
                 (bottom_right_x, bottom_right_y)])
            top_left_pixel = gray_img[top_left_y][top_left_x]
            top_right_pixel = gray_img[top_left_y][bottom_right_x]
            bottom_left_pixel = gray_img[bottom_right_y][top_left_x]
            bottom_right_pixel = gray_img[bottom_right_y][bottom_right_x]
            #print "corners_pixels=%d, %d, %d, %d" \
            #    % (top_left_pixel, top_right_pixel,
            #       bottom_left_pixel, bottom_right_pixel)
            eval_func[i] += (int(top_left_pixel) + int(top_right_pixel) + \
                             int(bottom_left_pixel) + int(bottom_right_pixel)) \
                             * self.expanded_corner_coef

            # calc distance from previous target centroid
            if not self.prev_centroid_area is None:
                eval_func[i] += (abs(blob_centroids[i][0] \
                                     - self.prev_centroid_area[0]) \
                                 + abs(blob_centroids[i][1] \
                                       - self.prev_centroid_area[1])) \
                                * self.prev_centroid_coef
                
            
        for i in range(num_labels-1):
            if (eval_func[i] < self.eval_func_thre) and \
               (eval_func[i] != -1):
                blob_idxs.append(i)
                if prev_bdsize < blob_dims[i + 1][4]:
                    biggest_idx = i
                    prev_bdsize = blob_dims[i + 1][4]

        #print "blob_idxs="
        #print blob_idxs
        processed_image = cv2.resize(frame,
                                     (self.resized_img_width,
                                      self.resized_img_height))

        if biggest_idx is None:
            blob_centroid_area = None
            print "no biggest blob"

        
            
       # self.print_difftime()
       # if num_labels > self.max_blob_num:
           # blob_centroid_area = None
           # print "too many blobs. failed finding"

        # create processed_image
        colors = []
        for i in range(num_labels):
            colors.append(np.array([255,0,0]))
        self.print_difftime()
        for y in range(self.resized_img_height):
            for x in range(self.resized_img_width):
                xx = x * self.orig_img_width / self.resized_img_width
                yy = y * self.orig_img_height / self.resized_img_height
                if label_image[yy, xx] > 0:
                    processed_image[y, x] = colors[label_image[yy, xx]]
        if (not mid_size_blob_idxs is None) and (not blob_centroids is None):
            blob_centroids_uint = np.uint16(np.around(blob_centroids))
            for i in mid_size_blob_idxs:
                cv2.circle(processed_image,
                           (blob_centroids_uint[i][0]
                            * self.resized_img_width
                            / self.orig_img_width,
                            blob_centroids_uint[i][1]
                            * self.resized_img_width
                            / self.orig_img_width),
                           1,
                           (0, 0, 255), 2)
        if (not expanded_rects_corners is None):
            for erc in expanded_rects_corners:
                cv2.rectangle(processed_image,
                              tuple(np.array(erc[0])
                                   * self.resized_img_width /
                                   self.orig_img_width),
                              tuple(np.array(erc[1])
                                   * self.resized_img_width /
                                   self.orig_img_width),
                              (0, 255, 0), 1)
        if (not blob_idxs is None) and (not blob_centroids is None):
            for bi in blob_idxs:
                cv2.circle(processed_image,
                           (blob_centroids_uint[bi][0]
                            * self.resized_img_width / self.orig_img_width,
                            blob_centroids_uint[bi][1]
                            * self.resized_img_width / self.orig_img_width),
                           1,
                           (200, 0, 255), 2)
                           
        if (not biggest_idx is None) and (not blob_centroids is None):
            cv2.circle(processed_image,
                       (blob_centroids_uint[biggest_idx][0]
                        * self.resized_img_width / self.orig_img_width,
                        blob_centroids_uint[biggest_idx][1]
                        * self.resized_img_width / self.orig_img_width),
                       3,
                       (0, 255, 255), 2)
        if (len(too_big_idxs) != 0) and (not blob_centroids is None):
            for idx in too_big_idxs:
                cv2.circle(processed_image,
                           (blob_centroids_uint[idx][0]
                            * self.resized_img_width / self.orig_img_width,
                            blob_centroids_uint[idx][1]
                            * self.resized_img_width / self.orig_img_width),
                           5,
                           (255, 0, 255), 2)
        self.print_difftime()
        if not biggest_idx is None:
            blob_centroid_area = np.concatenate(
                (blob_centroids[biggest_idx],
                 [blob_dims[biggest_idx + 1][4]]))
        del gray_img
        return processed_image, blob_centroid_area, biggest_idx, too_big_idxs

    
    def publish_processed_image(self, processed_image):
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))

        
    def publish_vel(self, blob_centroid_area, biggest_idx, too_big_idxs):
        vel = Twist()
        
        if self.move_flag == False:
            if not (self.prev_vel == Twist()):
                self.vel_pub.publish(vel) # 0 velocity
                self.prev_vel = vel
            return
        
        if blob_centroid_area is None:
            if (self.prev_biggest_idx in too_big_idxs) and \
               (self.prev_xvel > 0):
                self.keep_move_forward_counter = self.keep_move_forward_init
            if self.keep_move_forward_counter > 0:
                #vel.linear.x = self.xvel_coef
                vel.linear.x = 0
                self.keep_move_forward_counter -= 1
                self.keep_move_forward_finished = False 
                self.vel_pub.publish(vel)
                self.prev_vel = vel
                self.prev_biggest_idx = biggest_idx
                self.prev_xvel = vel.linear.x
                return
            else:
                if (not self.arrival_flag is True) and \
                   (self.keep_move_forward_finished is False):
                    self.keep_move_forward_finished = True
                    #self.move_flag = False
                    self.arrival_flag = True
                    rospy.loginfo('arrived2')
                    print 'arrived2'
                    vel = Twist()
                    self.vel_pub.publish(vel)  # 0 velocity
                    self.prev_vel = vel
                    self.prev_biggest_idx = biggest_idx
                    return
                print 'not publish velocity'
                vel = Twist()
                self.vel_pub.publish(vel) # 0 velocity
                self.prev_vel = vel
                return
        if abs((self.orig_img_width / 2) - blob_centroid_area[0]) \
           < self.center_line_thre:
            if self.prev_vel.linear.y != 0:
                vel.linear.x = 0
                self.vel_pub.publish(vel)
                self.prev_vel = vel
                self.prev_biggest_idx = biggest_idx
                self.prev_xvel = vel.linear.x
                time.sleep(0.7)
            else:
                vel.linear.x = self.xvel_coef
                self.vel_pub.publish(vel)
                self.prev_vel = vel
                self.prev_biggest_idx = biggest_idx
                self.prev_xvel = vel.linear.x

                
        else:
            if (self.prev_biggest_idx in too_big_idxs) and \
               (self.prev_xvel > 0):
                self.keep_move_forward_counter = self.keep_move_forward_init
                
            if self.keep_move_forward_counter > 0:
                #vel.linear.x = self.xvel_coef
                vel.linear.x = 0
                self.keep_move_forward_counter -= 1
                self.keep_move_forward_finished = False
                self.vel_pub.publish(vel)
                self.prev_vel = vel
                self.prev_biggest_idx = biggest_idx
                self.prev_xvel = vel.linear.x

            else:
                if (not self.arrival_flag is True) and\
                   (self.keep_move_forward_finished is False):
                    self.keep_move_forward_finished = True
                    #self.move_flag = False
                    self.arrival_flag = True
                    rospy.loginfo('arrived1')
                    print 'arrived1'
                    vel = Twist()
                    self.vel_pub.publish(vel)  # 0 velocity
                    self.prev_vel = vel
                    self.prev_biggest_idx = biggest_idx
                    return

                #yvel = ((self.orig_img_width / 2) - blob_centroid_area[0]) \
                #       * self.yvel_coef
                if ((self.orig_img_width / 2) - blob_centroid_area[0]) >= 0:
                    yvel = self.yvel_coef
                else:
                    yvel = -1 * self.yvel_coef

                if self.prev_vel.linear.x != 0:
                    vel.linear.y = 0
                    self.vel_pub.publish(vel)
                    self.prev_vel = vel
                    self.prev_biggest_idx = biggest_idx
                    self.prev_xvel = vel.linear.x
                    time.sleep(0.7)
                else:
                    vel.linear.y = yvel
                    #vel.angular.z = yvel
                    self.vel_pub.publish(vel)
                    self.prev_vel = vel
                    self.prev_biggest_idx = biggest_idx
                    self.prev_xvel = vel.linear.x


    def cleanup(self):
        print "shutting down"
        cv2.destroyAllWindows()

        
if __name__ == '__main__':
    try:
        #findsphere = FindSphere(image_type = FindSphere.VIDEO)
        findsphere = FindSphere(image_type = FindSphere.CAMERA)
        #findsphere = FindSphere(image_type = FindSphere.STATIC_IMAGE)
        rate = rospy.Rate(5)
        prevtime = time.time()
        while not rospy.is_shutdown():
            findsphere.capture_and_process()
            rate.sleep()
            curtime = time.time()
            #print 'laptime=%f' % (curtime - prevtime)
            prevtime = curtime
    except KeyboardInterrupt:
        sys.exit()
