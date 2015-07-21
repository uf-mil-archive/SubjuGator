#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from txros import util
from sub8_vision_arbiter.msg import *

LOWER_YELLOW = np.array([15,50,50])
UPPER_YELLOW = np.array([40,255,255])

LOWER_RED = np.array([0,50,50])
UPPER_RED = np.array([10,255,255])

class find_signs(object):
    def __init__(self):

        self.master_import = None #cv2.imread('IMG_1533.JPG', cv2.IMREAD_COLOR)
        self.master_hsv = None
        self.yellow_area = 0.0
        self.x_y_added = []
        self.bridge = CvBridge()
        self.RUN_VISION = False
        self.area_pub = rospy.Publisher("torpedo/area", Float64, queue_size = 1)
        rospy.Subscriber("/forward_camera/image_color", Image , self.update_image)
        rospy.Subscriber("/vision_arbiter", vision_arbiter, self.update_vision)
        rospy.spin()

    def update_vision(self,msg):
        self.RUN_VISION = msg.torpedo_area_vision

    def get_torpedo_area(self):

        yellow_mask = cv2.inRange(self.master_hsv, LOWER_YELLOW, UPPER_YELLOW)
        # Overlay yellow onto full image

        count = 0.0
        for i in range(0, self.height):
            for j in range(0,self.width):
                if yellow_mask[i][j] == 255:
                    count += 1

        print count

        yellow_area = (count / (a640.0*480.0))
        self.area_pub.publish(yellow_area)
        print yellow_area

    def update_image(self, msg):

        if self.RUN_VISION == True:

            try:
              vid = self.bridge.imgmsg_to_cv2(msg, "bgr8")
              self.master_import = vid
            except CvBridgeError, e:
              print e    

            self.height, self.width, self.chan = self.master_import.shape
            self.master_hsv = cv2.cvtColor(self.master_import, cv2.COLOR_BGR2HSV)

            self.get_torpedo_area()

if __name__ == "__main__":
    rospy.init_node('torpedo_vision')
    find_signs = find_signs()

    

    


    

    
