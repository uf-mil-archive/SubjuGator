#! usr/bin/env pyhon

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point

LOWER_YELLOW = np.array([20,50,50])
UPPER_YELLOW = np.array([40,255,255])

LOWER_RED = np.array([0,50,50])
UPPER_RED = np.array([10,255,255])

class find_signs(object):
	def __init__(self):

		self.master_import = cv2.imread('IMG_1533.JPG', cv2.IMREAD_COLOR)
		self.height, self.width, self.chan = self.master_import.shape
		self.master_hsv = cv2.cvtColor(self.master_import, cv2.COLOR_BGR2HSV)

		self.yellow_overlay = None
		self.red_overlay = None
		self.yellow_mask = None
		self.red_mask = None
		self.final_shapes = None

		self.x_pixel_array = []
		self.y_pixel_array = []

		# Convert to set to remove duplicatesss
		self.x_set = set()
		self.y_set = set()

		self.sign_x_center = 0
		self.sign_y_center = 0

		self.top_right_pub = rospy.Publisher("torpedo/TR", Point, queue_size = 1)
		self.top_left_pub = rospy.Publisher("torpedo/TL", Point, queue_size = 1)
		self.bottom_right_pub = rospy.Publisher("torpedo/BR", Point, queue_size = 1)
		self.bottom_left_pub = rospy.Publisher("torpedo/BL", Point, queue_size = 1)
		self.center_publisher = rospy.Publisher("torpedo/center", Point, queue_size = 1)

	def red_work(self):
		self.red_mask = cv2.inRange(self.master_hsv, LOWER_RED, UPPER_RED)
		self.red_overlay = cv2.bitwise_and(self.master_import, self.master_import, mask = self.red_mask)

	def yellow_work(self):
		self.yellow_mask = cv2.inRange(self.master_hsv, LOWER_YELLOW, UPPER_YELLOW)
		self.yellow_overlay = cv2.bitwise_and(self.master_import, self.master_import, mask = self.yellow_mask)

	def filter_and_assign(self):
		combined = cv2.bitwise_or(self.red_mask, self.yellow_mask)
		combined_blur = cv2.GaussianBlur(combined,(5,5),0)
		_,thres = cv2.threshold(combined_blur,140,255,cv2.THRESH_BINARY)
		adapt_thres = cv2.adaptiveThreshold(combined_blur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11,2)
		gaus_blur = cv2.GaussianBlur(combined_blur,(5,5),0)
		_,otsu_thres = cv2.threshold(gaus_blur,200,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
		self.final_shape = cv2.bitwise_and(thres, adapt_thres , self.red_mask, self.yellow_mask)
		self.final = cv2.bitwise_and(self.master_import,self.master_import, mask = self.final_shape)

	def contour_then_filter(self):

		contours, heirarchy = cv2.findContours(self.final_shape,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for x in xrange(1,len(contours)):
			if heirarchy[0][x] != None:
				temp = heirarchy[0][x]
				if temp[3] != -1:
					cv2.drawContours(self.final, contours, x, (0,255,0), 3)
					cnt = contours[x]
					M = cv2.moments(cnt)
					area = cv2.contourArea(cnt)
					if M['m10'] and M['m00'] and M['m01'] and M['m00'] and area > 30000 != 0:
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						cv2.circle(self.master_import,(cx,cy),10,(255,0,0),-1)
						self.x_pixel_array.append(cx)
						self.y_pixel_array.append(cy)
						self.x_set = self.x_pixel_array
						self.y_set = self.y_pixel_array

	def output(self):
		x_temp = 0
		y_temp = 0
		for x in self.x_set:
			x_temp += x
		for y in self.y_set:
			y_temp += y

		self.sign_x_center = x_temp/8
		self.sign_y_center = y_temp/8

		cv2.circle(self.master_import,(self.sign_x_center, self.sign_y_center),10,(0,0,255),-1)
		to_pub = Point(x = self.sign_x_center, y = self.sign_y_center)
		self.center_publisher.publish(to_pub)

		cv2.imshow('res',self.master_import)
		k = cv2.waitKey(0)

if __name__ == "__main__":
	rospy.init_node('torpedo_vision')
	find_signs = find_signs()
	find_signs.red_work()
	find_signs.yellow_work()
	find_signs.filter_and_assign()
	find_signs.contour_then_filter()
	find_signs.output()
	
	

	


	

	

