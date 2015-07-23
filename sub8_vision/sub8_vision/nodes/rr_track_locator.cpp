#include <iostream>
#include <stdlib.h>
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

ros::Publisher tracks_pub;
bool rr_track_locator_node_switch = false;

// We need a global variable n order to use the slider in our callback function
int max_corners_slider;
int min_dist_slider;

void nodeToggler(const sub8_vision_arbiter::vision_arbiter::ConstPtr& msg){
	rr_track_locator_node_switch = msg->tracks_vision;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg){

	if(rr_track_locator_node_switch == false) return;

	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const double RE_UPSAMPLING_FACTOR = 1/DOWNSAMPLING_FACTOR;
	const int RECOGNITION_RADIUS = 100;		// Only recognizes objects within this distance from origin

	// Image containers
	Mat currentFrame, outputFrame, cornerDetectImg, floodFillMask, contourExtractImg;

	//Node Handle
	ros::NodeHandle n;

	// Publishers
	ros::Publisher tracks_pub = n.advertise<geometry_msgs::Point>("train_tracks", 1000);

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

	// Set the center of the downsampled frame as the frame origin
	const Point camCenter = Point(currentFrame.cols / 4, currentFrame.rows / 4);

	// Downsampling and blurring
	resize(currentFrame, outputFrame, Size(0, 0), DOWNSAMPLING_FACTOR, DOWNSAMPLING_FACTOR, INTER_NEAREST);
	Size kernelSize = Size(3, 3);
	GaussianBlur(outputFrame, outputFrame, kernelSize, 0);
	Mat readOnlySrcColor = outputFrame.clone();
	Mat readOnlySrcGray;
	cvtColor(outputFrame, readOnlySrcGray, CV_BGR2GRAY);

	imshow("Input", outputFrame); // DBG
	waitKey(300);
	

/*
	// *DBG*
	cout << "outputFrame dimensions:\n";
	cout << "	rows = " << outputFrame.rows << endl;
	cout << "	cols = " << outputFrame.cols << endl;
*/

	// Converting to HSV colorspace and isolating Hue and Sat channels

	vector<Point2f> corners;
	double qualityLevel = 0.01;
	double minDistance = min_dist_slider;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k =  0.04;



	// Apply corner detection
	goodFeaturesToTrack(readOnlySrcGray,
				corners,
				max_corners_slider,
				qualityLevel,
				minDistance,
				Mat(),
				blockSize,
				useHarrisDetector,
				k);

	
	//Initialize needed matrices
	cornerDetectImg = Mat(readOnlySrcGray.size(), CV_8UC1, Scalar(0));
	contourExtractImg = Mat(readOnlySrcGray.size(), CV_8UC1, Scalar(0));
	floodFillMask = Mat(readOnlySrcGray.size() + Size(2,2), CV_8UC1, Scalar(0)); // adds an all around pixel wide border
	Mat mask_rect = floodFillMask.clone(); 

/*
	// *DBG*
	cout << "floodFillMask dimensions:\n";
	cout << "	rows = " << floodFillMask.rows << endl;
	cout << "	cols = " << floodFillMask.cols << endl;
*/

 	cout << "** Number of corners detected: " << corners.size()<< endl;

 	// Draw corners detected
  	int r = 2; // dot radius
  	for( int i = 0; i < corners.size(); i++ ){ 
  		//circle( outputFrame, corners[i], r, Scalar(122), -1, 8, 0 );
	    circle( cornerDetectImg, corners[i], r, Scalar(255), -1, 8, 0);
	}
	imshow("Corners", cornerDetectImg); //waitKey(0);
	

	// Calculate centroid and standard deviation of detected points
	double xSum = 0;
	double ySum = 0;
	double ptCount = 0;
	for (int i = 0; i < corners.size(); i++){
		xSum += corners[i].x;
		ySum += corners[i].y;
		ptCount++;
	}
	Point2f prelimCentroid = Point2f(-1,-1); 
	if (ptCount > 0) prelimCentroid= Point2f(xSum/ptCount,ySum/ptCount);

	// Create a rectangle around the centroid
	int crWidth = 30;
	int crHeight = 30;
	int crX = (int)prelimCentroid.x - crWidth/2;
	int crY = (int)prelimCentroid.y - crHeight/2;
	Rect centerRect = Rect(crX, crY, crWidth, crHeight);
	rectangle(cornerDetectImg, centerRect, Scalar(255));

/*
	TODO: 
	* calculate avg distance from centroid;
	* eliminate corner points that are too far from the centroid
	* calculate new centroid
*/

	// Sharpen img to prepare for floodfill
	Mat sharpened = readOnlySrcGray.clone();
	unsharpMask(sharpened);
	imshow("Sharpened", sharpened); // DBG

	// Iterate through pixels inside the rect and floodfill a mask of contiguous dark pixels
	bool floodFilled = false;
	for(int y = centerRect.y; y < centerRect.y + centerRect.height; y += 3){
		for(int x = centerRect.x; x < centerRect.x + centerRect.width; x += 2){
			Point currentPt = Point(x, y);
			//cout << "currentPt: " << currentPt << "	Val: " << (int)sharpened.at<uchar>(currentPt) << endl;
			if (sharpened.at<uchar>(currentPt) < 60){
				floodFill(sharpened, floodFillMask, currentPt, Scalar(), 0, Scalar(10), Scalar(8), 8 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
				floodFilled = true;
				cout << "floodfill successful!\n"; break;
			}
		}
	}
	if (!floodFilled) return;
	cout << "mask area: " << maskArea(floodFillMask) << endl;

	imshow("Tracks Mask", floodFillMask); // DBG


	// Create mask of dark pixels using blackFilter()
	Mat brightnessThreshImg = Mat(outputFrame.size(), CV_8UC1, Scalar(0));
	//threshold(readOnlySrcGray, brightnessThreshImg, 40, 255, THRESH_BINARY_INV);
	brightnessThreshImg = blackFilter(readOnlySrcColor); cout << "black filter complete\n";
	imshow("black filter", brightnessThreshImg);

	// Extract contours from floodFill mask
	vector<vector<Point>> contours;
	contourExtractImg = floodFillMask(Rect(1,1, outputFrame.cols, outputFrame.rows));
	findContours(contourExtractImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Find index of the largest contour
	int largestContIdx = 0;
	if(contours.size() > 1){
		int largestSizeYet = contours[0].size();
		for(int i = 0; i < contours.size(); i++){
			if(contours[i].size() > largestSizeYet){
				largestContIdx = i;
				largestSizeYet = contours[i].size();
			}
		}
	}

	// Draw what we expect to be the outside track contours on debugging imgs
	drawContours(mask_rect, contours, largestContIdx, Scalar(255), 1);
	drawContours(outputFrame, contours, largestContIdx, Scalar(255), 1);
	

	// Calculate the track countour's minimum enclosing rect
	RotatedRect minArea = minAreaRect(contours[largestContIdx]);
	
	// Draw min area rect
	Point2f vertices[4];
	minArea.points(vertices);
	for (int i = 0; i < 4; i++){
		if (i == 3) line(mask_rect, vertices[i], vertices[0], Scalar(255), 2);
		else line(mask_rect, vertices[i], vertices[i + 1], Scalar(255), 2);
	}
	imshow("mask_rect", mask_rect);

	// Put debug text on debug imgs
	float track_orientation_deg = trainTracksLeastAngle(minArea);
	std::stringstream to_deg;
	std::stringstream to_pos;
	to_deg << "Angle: " << (int)track_orientation_deg << " deg";
	to_pos << "Pose: " << "(" << (int)(minArea.center.x * RE_UPSAMPLING_FACTOR) << ", " << (int)(minArea.center.y * RE_UPSAMPLING_FACTOR) << ")";
	string debugImgTxt =  to_deg.str();
	string debugImgTxt2 =  to_pos.str();
	putText(outputFrame, debugImgTxt, Point(5, 20), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,0), 2);
	putText(outputFrame, debugImgTxt2, Point(5, 45), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,255,0), 2);

	// Publish train track pose and orientation
	geometry_msgs::Point track_center_orientation;
	track_center_orientation.x = minArea.center.x * RE_UPSAMPLING_FACTOR;
	track_center_orientation.y = minArea.center.y * RE_UPSAMPLING_FACTOR;
	track_center_orientation.z = track_orientation_deg;
	tracks_pub.publish(track_center_orientation);

	// Display final debug imgs
	circle( outputFrame, prelimCentroid, 4, Scalar(255), -1, 8, 0);
	imshow("Visuals", outputFrame); // DBG
	// imshow("Val Img", frameVAL); // DBG
}	

int main(int argc, char* argv[]){
	ros::init(argc, argv, "rr_track_locator");

 	ros::NodeHandle n;

 // 	Mat tracksImg = imread("/home/santiago/Downloads/train_tracks.bmp", CV_LOAD_IMAGE_GRAYSCALE);
 // 	waitKey(5000);
 // 	if( !tracksImg.data )
	// {
 //   		printf( " No image data \n " );
 //   		return -1;
 // 	}
 // 	waitKey(5000);
 // 	imshow("Tracks", tracksImg);
 // 	waitKey(0);
 // 	for(int y = 0; y < tracksImg.rows; y++){
 // 		for(int x = 0; x < tracksImg.cols; x++){
 // 		int pixVal = tracksImg.at<uchar>(y, x);
 // 		if (pixVal < 122) tracksImg.at<uchar>(y, x) = 0;
 // 		else tracksImg.at<uchar>(y, x) = 255;
 // 		imshow("Polished Tracks", tracksImg);
 // 		waitKey(40);
 // 		}
 // 	}
 // 	imshow("Polished Tracks", tracksImg);
 // 	waitKey(0);
 // 	return 0;
 	


 	// Create and initialize slider and its window
 	namedWindow("Corners");
 	max_corners_slider = 50;
 	min_dist_slider = 1;
 	createTrackbar("Max Corners: ", "Corners", &max_corners_slider, 100);
 	createTrackbar("Min Distance: ", "Corners", &min_dist_slider, 50);

 	// Subscribe to node activation topic
 	ros::Subscriber node_activation_sub = n.subscribe("/vision_arbiter",1,nodeToggler);
 	
	// Since we're subscribing to an image, use an image_transport::Subscriber
	image_transport::ImageTransport it(n);
 	image_transport::Subscriber cam_sub = it.subscribe("/forward_camera/image_color",\
		 1, imgCallback); // Needs to be changed to downward camera

 	ros::spin();

 	return 0;
}