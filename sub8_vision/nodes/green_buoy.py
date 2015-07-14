#!/usr/bin/env python

import roslib

import sys
import rospy
import cv2
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
'''TODO 
   -Publish image message with visual aids
   -Make values dynamicly adaptive
      -constants for image size 
      -Adaptive thresholding 
'''
class image_converter:

  def __init__(self):
    self.angle_pub = rospy.Publisher("green_buoy_vision", Point, queue_size=1)
    self.image_pub = rospy.Publisher("green_buoy_debug",Image)


    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/forward_camera/image_rect_color", Image, self.callback)

  
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    cv_image2 = cv_image
    img = cv_image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#HSV is good for color detection with lighting flux  
    #thresholding and building heat mask



    def find_green_buoy(img):
        h1 = 51
        s1 = 0
        v1 = 0
        h2 = 83
        s2 = 255
        v2 = 255
        lower_value = np.array([h1, s1, v1], np.uint8)
        upper_value = np.array([h2, s2, v2], np.uint8)
        #endthresholding
        temp = np.array(0)
        cv2.rectangle(img,(0,0),(960,200),(0,0,0),-1)        #kernel = np.ones((150,150),np.uint8)
        mask = cv2.inRange(img, lower_value, upper_value)
        bImg = cv2.bitwise_or(mask, temp)
        #cv2.rectangle(thresh,(0,0),(960,200),(0,0,0),-1)        #kernel = np.ones((150,150),np.uint8)
        #bImg = cv2.morphologyEx(bImg, cv2.MORPH_CLOSE, kernel)#messed something up was included in a tutorial not entirely required here
        ret, thresh = cv2.threshold(bImg, 127, 255, 0)
        #end thresholding and building heat mask
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_area_quick_sort(contours)
        if len(contours)>0:
            moment0 = cv2.moments(contours[0])
            if moment0['m00']==0: return 0
            cx0 = int(moment0['m10']/moment0['m00'])
            cy0 = int(moment0['m01']/moment0['m00'])
            return [cx0, cy0, 0]
        else: return None
        

    def contour_area_quick_sort(l):
        length = len(l)
        if length <=1:
            return l
        else:
            pivot = l.pop(int(length/2))
            less, more = [], []
            for x in l:
                if cv2.contourArea(x)>=cv2.contourArea(pivot):
                    less.append(x)
                else:
                    more.append(x)
            return contour_area_quick_sort(less) + [pivot] + contour_area_quick_sort(more)    
    
    #cv_image2 = cv2.line(cv_image2,(cx0,0),(cx0,480),(0,0,255),5)
    #cv_image2 = cv2.line(cv_image2,(cx1,0),(cx1,480),(0,255,0),5)
    #contours = cv2.findContours(thresh, 1, 2)#findimg contours really just masses of colors
    #img = cv2.drawContours(img, contours, 0, (0,0,0), 3)
    #img = cv2.drawContours(img, contours, 1, (0,0,0), 3)
    
    green_buoy_point = find_green_buoy(img)
    cv_image2 = cv2.line(cv_image2,(0,0),(green_buoy_point[0],green_buoy_point[1]),(255,0,0),5)
    print 'green_buoy_point', green_buoy_point
    #print 'biggest', cx0, 'second', cx1     
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e
    

    #cnt = contours[0]                        #selecting main contour    
    #x,y,w,h = cv2.boundingRect(cnt)
    #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
    #cv2.imshow("Gates",img)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    if green_buoy_point != None:
        green_buoy_point[0]=green_buoy_point[0]-320
        green_buoy_point[1]=green_buoy_point[1]-240        
        self.angle_pub.publish(Point(*green_buoy_point))
  
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('green_buoy_vision')
    main(sys.argv)
