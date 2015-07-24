#include <iostream>
#include "vision_tools.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sub8_vision_arbiter/vision_arbiter.h>

#define PI        3.14159265358979323846


using namespace std;
using namespace cv;

ros::Publisher portal_pub;
bool portal_pub_node_switch = false;

// We need a global variable n order to use the slider in our callback function
int rec_viz_thresh_slider = 140;

void nodeToggler(const sub8_vision_arbiter::vision_arbiter::ConstPtr& msg){
	portal_pub_node_switch = msg->tracks_vision;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg){

	if(portal_pub_node_switch == false) return;

	// Constants
	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const double RE_UPSAMPLING_FACTOR = 1/DOWNSAMPLING_FACTOR;
	const int RECOGNITION_RADIUS = 100;		// Only recognizes objects within this distance from origin

	// Image containers
	Mat currentFrame, frameHSV, frameHUE, frameSAT, blobExtractionImg, outputFrame;

	//Node Handle
	ros::NodeHandle n;

	// Publishers
	ros::Publisher portal_pub = n.advertise<geometry_msgs::Point>("portal_center", 1000);
	
	usleep(1000*100);

	// Get current frame
	cv_bridge::CvImagePtr img_ptr;
    try
    {
    	img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	currentFrame = img_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }
    catch (const std::exception& e) {
    	ROS_ERROR("Exception: %s", e.what());
    	return;
    }

    // Downsampling and blurring
	resize(currentFrame, outputFrame, Size(0, 0), DOWNSAMPLING_FACTOR, DOWNSAMPLING_FACTOR, INTER_NEAREST);
	Size kernelSize = Size(3, 3);
	GaussianBlur(outputFrame, outputFrame, kernelSize, 0);

	// Containers for color conversion
	frameHSV = Mat::zeros(outputFrame.size(), CV_8U);
	vector<Mat> separtedHSV(3);
	vector<vector<Point>> HSVcontours;

	// Converting to HSV colorspace and isolating Hue and Sat channels
	cvtColor(outputFrame, frameHSV, CV_BGR2HSV);
	split(frameHSV, separtedHSV);
	frameHUE = separtedHSV[0].clone();
	frameSAT =  separtedHSV[1].clone();
	Mat frameVAL =  separtedHSV[2].clone(); // DBG

	// Threshold saturation to segment colored objects from background
	threshold(frameSAT, blobExtractionImg, rec_viz_thresh_slider, 255, THRESH_BINARY);

	// Containers for blob extraction
	vector<vector<Point>> satThreshContours;

	// Blob extraction
	findContours(blobExtractionImg, satThreshContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Contain contour selection state
	vector<bool> cntrSelected(satThreshContours.size(), false);
	vector<double> cntrLengths(satThreshContours.size(), false);


	// Select contours of intrest
	for(int i = 0; i < satThreshContours.size(); i++){
		cntrLengths.push_back(arcLength(satThreshContours[i], false));
	}
	double lrgstLength = 0;
	double lrgst2ndLength = 0;
	int lrgst_1st_idx = 0;
	int lrgst_2nd_idx = 1;

	for(int i = 0; i < satThreshContours.size(); i++){
		if(cntrLengths[i] > lrgstLength) {
			lrgstLength = cntrLengths[i];
			lrgst_1st_idx = i;
		}
	}
	cntrSelected[lrgst_1st_idx] = true;


	if(lrgst_1st_idx != 0) lrgst_2nd_idx = 0;

	for(int i = 0; i < satThreshContours.size(); i++){
		if(cntrLengths[i] > lrgst2ndLength && i != lrgst_1st_idx) {
			lrgst2ndLength = cntrLengths[i];
			lrgst_2nd_idx = i;
		}
	}
	cntrSelected[lrgst_2nd_idx] = true;

	// Create bounding rectangles
	vector<RotatedRect> boundingRects(2);
	boundingRects.push_back(minAreaRect(satThreshContours[lrgst_1st_idx]));
	boundingRects.push_back(minAreaRect(satThreshContours[lrgst_2nd_idx]));

	// Calculate portal center coordinate
	int x = (boundingRects[0].center.x + boundingRects[1].center.x) / 2;
	int y = (boundingRects[0].center.y + boundingRects[1].center.y) / 2;

	// Publish results
	geometry_msgs::Point portal_center;
	portal_center.x = x * RE_UPSAMPLING_FACTOR;
	portal_center.y = y * RE_UPSAMPLING_FACTOR;
	portal_pub.publish(portal_center);

}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "portal_vision");

 	ros::NodeHandle n;

 	// Create and initialize slider
 	//namedWindow("Thresholding");
 	rec_viz_thresh_slider;
	//createTrackbar("Saturation Threshold: ", "Thresholding", &rec_viz_thresh_slider, 255);

	// Subscribe to node activation topic
 	ros::Subscriber node_activation_sub = n.subscribe("/vision_arbiter",1,nodeToggler);

	// Since we're subscribing to an image, use an image_transport::Subscriber
	image_transport::ImageTransport it(n);
 	image_transport::Subscriber cam_sub = it.subscribe("/forward_camera/image_color",\
		 1, imgCallback); // Needs to be changed to downward camera

 	ros::spin();

 	return 0;
}