#!/usr/bin/env python
'''THIS IS LUCAS'S FILE GO TO HIM WITH QUESTIONS lbassettaudain AT ufl DOT edu
'''
import roslib
#roslib.load_manifest('start_gate_vision')
import sys
import rospy
import cv2
from std_msgs.msg import Float64
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
    self.angle_pub = rospy.Publisher("yellow_upright", Float64, queue_size=1)
    self.image_pub = rospy.Publisher("yellow_upright_debug",Image)


    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

  
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    cv_image2 = cv_image
    img = cv_image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#HSV is good for color detection with lighting flux  
    #thresholding and building heat mask
    def find_yellow_upright(img):
        h1 = 22
        s1 = 0
        v1 = 0
        h2 = 83
        s2 = 255
        v2 = 255
        lower_value = np.array([h1, s1, v1], np.uint8)
        upper_value = np.array([h2, s2, v2], np.uint8)
        #endthresholding
        temp = np.array(0)
        mask = cv2.inRange(img, lower_value, upper_value)
        bImg = cv2.bitwise_or(mask, temp)
        #img = cv2.rectangle(img,(0,0),(960,200),(0,0,0),-1)
        ret, thresh = cv2.threshold(bImg, 127, 255, 0)
        #end thresholding and building heat mask
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_area_quick_sort(contours)
        if len(contours)>0:
            moment0 = cv2.moments(contours[0])
            cx0 = int(moment0['m10']/moment0['m00'])
            #cy0 = int(moment0['m01']/moment0['m00'])
            if cx0 < 320:
                angle_sign = 1
            else:
                angle_sign = -1
            angle = cv2.contourArea(contours[0])/cv2.contourArea(contours[1])*angle_sign
            print 'ANGLE', angle
            self.angle_pub.publish(Float64(data=angle))
            return angle
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
    

    
    angle = 
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
        
    
  
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('start_gate_vision')
    main(sys.argv)