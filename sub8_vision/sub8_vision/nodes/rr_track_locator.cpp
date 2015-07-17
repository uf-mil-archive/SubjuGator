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

#define PI        3.14159265358979323846


using namespace std;
using namespace cv;

ros::Publisher rr_tracks_pub;

// We need a global variable n order to use the slider in our callback function
int corner_thresh_slider;

void imgCallback(const sensor_msgs::ImageConstPtr& msg){

	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const double RE_UPSAMPLING_FACTOR = 1/DOWNSAMPLING_FACTOR;
	const int RECOGNITION_RADIUS = 100;		// Only recognizes objects within this distance from origin

	// Image containers
	Mat currentFrame, frameHSV, frameHUE, frameSAT, frameVAL, blobExtractionImg, outputFrame;

	//Node Handle
	ros::NodeHandle n;

	// Publishers
	ros::Publisher rr_tracks_pub = n.advertise<geometry_msgs::Point>("rr_tracks", 1000);

	// Wait for publisher subscriber connections to get set up
	// while (true){
	// 	bool readyToPublish = delorean_pub.getNumSubscribers() > 0 && train_pub.getNumSubscribers() > 0;
	// 	if (readyToPublish) break;
	// }
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

	imshow("Resized Frame", outputFrame); // DBG
	waitKey(10);
	cout << "line: " << 74 << endl;
	// Containers for color conversion
	vector<Mat> separtedHSV(3);
	vector<vector<Point>> HSVcontours;
	cout << "line: " << 78 << endl;
	// Converting to HSV colorspace and isolating Hue and Sat channels
	cvtColor(outputFrame, frameHSV, CV_BGR2HSV);
	split(frameHSV, separtedHSV);
	frameHUE =  separtedHSV[0].clone(); 
	frameSAT =  separtedHSV[1].clone(); 
	frameVAL =  separtedHSV[2].clone(); 
	cout << "line: " << 85 << endl;

	imshow("Hue Img", frameHUE); // DBG
	imshow("Sat Img", frameSAT); // DBG
	imshow("Val Img", frameVAL); // DBG
	cout << "line: " << 90 << endl;

	Sobel(frameVAL, blobExtractionImg, CV_8U, 1, 1);

	imshow("Eges (1st derivatives)", blobExtractionImg);

	Sobel(frameVAL, blobExtractionImg, CV_8U, 2, 2);

	imshow("Eges (2nd derivatives)", blobExtractionImg);

	// //threshold(frameVAL, blobExtractionImg, 120, 255, THRESH_BINARY);
	// //imshow("val thresh", blobExtractionImg); // DBG

	// findContours(frameVAL, HSVcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// cout << "line: " << 95 << endl;
	// drawContours(outputFrame, HSVcontours, -1, Scalar(0,0,255), 3);
	// cout << "line: " << 97 << endl;
	// waitKey(10);

	// imshow("Contours", outputFrame); // DBG

	Mat cornerDetectImg = Mat(outputFrame.size(), CV_8UC1, Scalar(0));
	// cornerHarris(frameVAL, cornerDetectImg, )









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
 	corner_thresh_slider = 0;
 	createTrackbar("Corner threshold:", "Corners", &corner_thresh_slider, 500);


	// Since we're subscribing to an image, use an image_transport::Subscriber
	image_transport::ImageTransport it(n);
 	image_transport::Subscriber cam_sub = it.subscribe("/down_camera/image_color",\
		 1, imgCallback); // Needs to be changed to downward camera

 	ros::spin();

 	return 0;
}