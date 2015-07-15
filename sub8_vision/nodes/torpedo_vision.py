#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

LOWER_YELLOW = np.array([20,50,50])
UPPER_YELLOW = np.array([40,255,255])

LOWER_RED = np.array([0,50,50])
UPPER_RED = np.array([10,255,255])

class find_signs(object):
    def __init__(self):

        self.master_import = None #cv2.imread('IMG_1533.JPG', cv2.IMREAD_COLOR)
        self.height, self.width, self.chan = 0,0,0
        self.master_hsv = None #cv2.cvtColor(self.master_import, cv2.COLOR_BGR2HSV)

        print self.height, self.width

        self.yellow_overlay = None
        self.red_overlay = None
        self.yellow_mask = None
        self.red_mask = None
        self.final_shapes = None

        self.x_set = set()
        self.y_set = set()
        

        self.x_y_added = []
        self.bridge = CvBridge()

        self.BL, self.TL, self.BR, self.TR, self.sign_center = 0,0,0,0,0

        self.top_right_pub = rospy.Publisher("torpedo/TR", Point, queue_size = 1)
        self.top_left_pub = rospy.Publisher("torpedo/TL", Point, queue_size = 1)
        self.bottom_right_pub = rospy.Publisher("torpedo/BR", Point, queue_size = 1)
        self.bottom_left_pub = rospy.Publisher("torpedo/BL", Point, queue_size = 1)
        self.center_publisher = rospy.Publisher("torpedo/center", Point, queue_size = 1)

        self.image_pub = rospy.Publisher("/torpedo_camera_out", Image, queue_size = 1)
        rospy.Subscriber("/forward_camera/image_color", Image , self.update_image)
        rospy.spin()

    def update_image(self, msg):
        
        try:
          vid = self.bridge.imgmsg_to_cv2(msg, "bgr8")
          self.master_import = vid
        except CvBridgeError, e:
          print e    

        self.height, self.width, self.chan = vid.shape
        self.master_hsv = cv2.cvtColor(vid, cv2.COLOR_BGR2HSV)

        self.red_work()
        self.yellow_work()
        self.filter_and_assign()
        self.contour_then_filter()
        #self.filter_centers()
        if len(self.x_set) > 3:
            self.output()

    def red_work(self):
        # Mask for red colors
        self.red_mask = cv2.inRange(self.master_hsv, LOWER_RED, UPPER_RED)
        # Overlay red onto full image
        self.red_overlay = cv2.bitwise_and(self.master_import, self.master_import, mask = self.red_mask)

    def yellow_work(self):
        # Maks for yellow colors
        self.yellow_mask = cv2.inRange(self.master_hsv, LOWER_YELLOW, UPPER_YELLOW)
        # Overlay yellow onto full image
        self.yellow_overlay = cv2.bitwise_and(self.master_import, self.master_import, mask = self.yellow_mask)

    def filter_and_assign(self):
        # Combine yellow and red masks
        combined = cv2.bitwise_or(self.red_mask, self.yellow_mask)
        # Gaussian blur the combined images
        combined_blur = cv2.GaussianBlur(combined,(5,5),0)
        # Threshold the blur for specific values - IGNORE FIRST VARIABLE
        _,thres = cv2.threshold(combined_blur,140,255,cv2.THRESH_BINARY)
        # Adaptive hreshold the blur for specific values - IGNORE FIRST VARIABLE
        adapt_thres = cv2.adaptiveThreshold(combined_blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11,2)
        # Gaussian blur once again
        gaus_blur = cv2.GaussianBlur(combined_blur,(5,5),0)
        # Otsu threshold for specific values - IGNORE FIRST VARIABLE - NOT USING RIGHT NOW
        _,otsu_thres = cv2.threshold(gaus_blur,200,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # If all the thresholds gave a value as being the correct color, keep that color
        self.final_shape = cv2.bitwise_and(thres, adapt_thres , self.red_mask, self.yellow_mask)
        # Add colors back to final image
        self.final = cv2.bitwise_and(self.master_import,self.master_import, mask = self.final_shape)

    def contour_then_filter(self):
        # Find contours in the image
        contours, heirarchy = cv2.findContours(self.final_shape,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        x_pixel_array = []
        y_pixel_array = []

        for x in xrange(1,len(contours)):
            if heirarchy[0][x] != None:
                temp = heirarchy[0][x]
                if temp[3] != -1:
                    cv2.drawContours(self.final, contours, x, (0,255,0), 3)
                    cnt = contours[x]
                    M = cv2.moments(cnt)
                    area = cv2.contourArea(cnt)
                    if M['m10'] and M['m00'] and M['m01'] and M['m00'] and area > 500 != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        x_pixel_array.append(cx)
                        y_pixel_array.append(cy)
                        self.x_set = x_pixel_array
                        self.y_set = y_pixel_array
                        self.x_y_added.append(cx+cy)


    def filter_centers(self):
        # Filter to remove value points that are too close to one anther
        count = 0
        to_remove = []
        for x in range(1, len(self.x_y_added)): 
            # if values in array minues next values is less than ten
            if abs(self.x_y_added[x-1] - self.x_y_added[x]) < 10: 
                self.x_set.pop(x-count)
                self.y_set.pop(x-count)
                count += 1

    def output(self):

        x_temp = 0
        y_temp = 0
        # Add up x points to find center
        for x in self.x_set:
            x_temp += x
        # Add up y points to find center
        for y in self.y_set:
            y_temp += y

        if x_temp == 0: x_temp = 1
        if y_temp == 0: y_temp = 1
        if len(self.x_set) == 0: self.x_set = []
        if len(self.y_set) == 0: self.y_set = []
        # Find x and y center values
        x_middle = x_temp/len(self.x_set)
        y_middle = y_temp/len(self.y_set)

        # Create messages to publish all point values
        self.sign_center = Point(x = x_middle, y = y_middle , z = 0)
        self.BR = Point(x = self.x_set[0], y = self.y_set[0], z = 0)
        self.BL = Point(x = self.x_set[1], y = self.y_set[1], z = 0)
        self.TL = Point(x = self.x_set[2], y = self.y_set[2], z = 0)
        self.TR = Point(x = self.x_set[3], y = self.y_set[3], z = 0)

        print "Top left point\n", self.TL
        print "Top right point\n", self.TR
        print "Bottom left point\n", self.BL
        print "Bottom right point\n", self.BR 
        print "Center at", (self.sign_center.x, self.sign_center.y)

        # Print values onto image
        cv2.circle(self.master_import,(self.TL.x, self.TL.y),4,(255,0,0),-1)
        cv2.circle(self.master_import,(self.TR.x, self.TR.y),4,(255,0,0),-1)
        cv2.circle(self.master_import,(self.BL.x, self.BL.y),4,(255,0,0),-1)
        cv2.circle(self.master_import,(self.BR.x, self.BR.y),4,(255,0,0),-1)
        cv2.circle(self.master_import,(self.sign_center.x, self.sign_center.y),4,(0,0,255),-1)

        # Publish values to ROS
        self.center_publisher.publish(self.sign_center)
        self.top_left_pub.publish(self.TL)
        self.top_right_pub.publish(self.TR)
        self.bottom_left_pub.publish(self.BL)
        self.bottom_right_pub.publish(self.BR)


        cv2.imshow('res',self.master_import)
        k = cv2.waitKey(3)

        #image_message = self.bridge.cv2_to_imgmsg(self.master_import)
        #self.image_pub.publish(image_message, "bgr8")

        self.x_set = set()
        self.x_set.clear()
        self.y_set = set() 
        self.y_set.clear()

if __name__ == "__main__":
    rospy.init_node('torpedo_vision')
    find_signs = find_signs()

    

    


    

    

