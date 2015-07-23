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

ros::Publisher delorean_pub;
ros::Publisher train_pub;
ros::Publisher handle_pub;
ros::Publisher guide_strip_pub;

// We need a global variable n order to use the slider in our callback function
int rec_viz_thresh_slider;

void imgCallback(const sensor_msgs::ImageConstPtr& msg){

	//cout << "callback was called" << endl; //DBG

	// Constants
	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const double RE_UPSAMPLING_FACTOR = 1/DOWNSAMPLING_FACTOR;
	const int RECOGNITION_RADIUS = 100;		// Only recognizes objects within this distance from origin

	// Image containers
	Mat currentFrame, frameHSV, frameHUE, frameSAT, blobExtractionImg, outputFrame;

	//Node Handle
	ros::NodeHandle n;

	// Publishers
	ros::Publisher delorean_pub = n.advertise<geometry_msgs::Point>("delorean", 1000);
	ros::Publisher train_pub = n.advertise<geometry_msgs::Point>("train", 1000);
	ros::Publisher handle_pub = n.advertise<geometry_msgs::Point>("handle", 1000);
	ros::Publisher guide_strip_pub = n.advertise<geometry_msgs::Point>("guide_strip", 1000);
	//ros::Publisher tracks_pub = n.Publisher<Geometry_msgs::Point>("tracks", 1000)

	// Wait for publisher subscriber connections to get set up
	// while (true){
	// 	bool readyToPublish = delorean_pub.getNumSubscribers() > 0 && train_pub.getNumSubscribers() > 0;
	// 	if (readyToPublish) break;
	// }
	usleep(1000*100);


	// Named Windows	
	//namedWindow("Vehicle Detection Output");
	//startWindowThread();
	//namedWindow("HUE");
	//startWindowThread();
	//namedWindow("After Flood Fill");
	//startWindowThread();
	//namedWindow("first contours");
	//startWindowThread();

	// Enumeration
	enum class Recognized
	{
		None, DeLorean, Train
	};

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
	//ImgConverter ic;

	// Set the center of the downsampled frame as the frame origin
	const Point camCenter = Point(currentFrame.cols / 4, currentFrame.rows / 4);

	// Downsampling and blurring
	resize(currentFrame, outputFrame, Size(0, 0), DOWNSAMPLING_FACTOR, DOWNSAMPLING_FACTOR, INTER_NEAREST);
	Size kernelSize = Size(3, 3);
	GaussianBlur(outputFrame, outputFrame, kernelSize, 0);

	//cout << "reduced frame size:	" << outputFrame.cols << " x " << outputFrame.rows << endl; // DBG

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

	// // Applying flood fill to Hue channel
	// float meanHue = mean(frameHUE)[0];
	// float acceptableHueError = 20.0;
	// cout << meanHue << endl;
	// blobExtractionImg = frameHUE.clone();
	// for (int i = 0; i < blobExtractionImg.rows; i += 10){
	// 	for(int j = 0; j < blobExtractionImg.cols; j += 10){
	// 		Point seedPt = Point(j,i);
	// 		int pixHue = blobExtractionImg.at<uchar>(i,j);
	// 		if (fabs(meanHue - pixHue) <= acceptableHueError){
	// 			floodFill(blobExtractionImg, seedPt, Scalar(255), 0, 3, 3, 8);
	// 		}

	// 	}
	// }
	//floodFill(blobExtractionImg, Point(0, 0), Scalar(0), 0, 3, 3, 8);
	//int thresh = 110;
	threshold(frameSAT, blobExtractionImg, rec_viz_thresh_slider, 255, THRESH_BINARY);

	
	imshow("HUE", frameHUE); // DBG
	imshow("SAT", frameSAT); // DBG
	//imshow("VAL", frameVAL); // DBG
	//waitKey(10);
	imshow("Thresholding", blobExtractionImg); // DBG
	//waitKey(10);
	
	// Containers for blob extraction
	vector<vector<Point>> floodFillContours, joinedFFContours;

	// Blob extraction
	findContours(blobExtractionImg, floodFillContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// Joining Blobs that are close together
	blobExtractionImg = Mat::zeros(outputFrame.size(), CV_8U);
	double distThresh = 30; 												// Will join blobs that are closer than this
	drawContours(blobExtractionImg, floodFillContours, -1, Scalar(255), 2);	// Draw blob mask image
	//imshow("First Countours", blobExtractionImg); // DBG
	for (int i = 0; i < floodFillContours.size(); i++){
		for (int j = 0; j < floodFillContours.size(); j++){

			// Connect blob masks with lines on image
			Point p1 = floodFillContours[i][0];
			Point p2 = floodFillContours[j][0];
			if (Distance(p1.x, p1.y, p2.x, p2.y) < distThresh) line(blobExtractionImg, p1, p2, Scalar(255), 2);
		}
	}

	imshow("joined contours", blobExtractionImg); // DBG
	//waitKey(10);

	// Extract object contours from joined blobs
	findContours(blobExtractionImg, joinedFFContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	drawContours(blobExtractionImg, joinedFFContours, -1, Scalar(122), 2);
	//imshow("Second Countours", blobExtractionImg); // DBG
	
	// Approximate joined contours with rotated rectangles
	vector<RotatedRect> rotatedRects;
	for (vector<Point> contour : joinedFFContours){
		rotatedRects.push_back(minAreaRect(contour));
	}

	// Debugging Drawings
	for (RotatedRect rect : rotatedRects){	
	
		// Retrieve object corners
		Point2f rectCorners[4];
		rect.points(rectCorners);
		for (int i = 0; i < 4; i++){
			if (i == 3) line(blobExtractionImg, rectCorners[i], rectCorners[0], Scalar(255), 2);
			else line(blobExtractionImg, rectCorners[i], rectCorners[i + 1], Scalar(255), 2);
		}
		circle(outputFrame, rect.center, 3, Scalar(0, 255, 0), CV_FILLED);
	}
	//imshow("Recognition Objects", blobExtractionImg); // DBG

	// Containers for hue averaging and centroid calculation
	int avgHue, hueSum, pixCount, xSum, ySum;

	// Object recognition loop
	Recognized identifiedVehicle = Recognized::None; //  0 -> No vehicle identified
	bool objectsRecognized = false;			// Temp, want to replace this
	int joinedBlobArea = 0;
	for (RotatedRect currentObject : rotatedRects){

		// Make contour of min area rect
		vector<Point> minAreaRectContour;
		Point2f vertices[4];
		currentObject.points(vertices);
		for( int i = 0; i < 4; i++){
			minAreaRectContour.push_back(vertices[i]);
		}
		
		// Reset hue averaging counters
		avgHue = hueSum = pixCount = xSum = ySum = 0;

		// Extract basic object properties
		Point2f objectCenter = currentObject.center;
		Rect objectRectangle = currentObject.boundingRect();
		float distanceToOrigin = Distance(camCenter.x, camCenter.y, objectCenter.x, objectCenter.y);

		// Stop object recognition in current frame if object is too far
		if (distanceToOrigin > RECOGNITION_RADIUS) continue;

		// Stop recognition if contour is too big (happens when there's no object in the scene)
		if (objectRectangle.area() > .75 * outputFrame.cols * outputFrame.rows){
			continue;
		}

		// Stop Recognition of small blobs
		if (objectRectangle.area() < 100) continue;

		// Determine which of the original flood fill blobs were joined to make the current object
		vector<vector<Point>> componentBlobs;
		for (int i = 0; i < floodFillContours.size(); i++){

			// Test all original flood fill blobs
			vector<Point> currentBlob = floodFillContours[i];
			bool insideBoundingRect = true;
			for (Point p : currentBlob){

				// No blob that has a point outside of the object rectangle should be considered a component blob
				if (objectRectangle.contains(p) == false) {
					insideBoundingRect = false;
					break;
				}
			}

			// Store component blobs
			if (insideBoundingRect) componentBlobs.push_back(currentBlob);
		}

		// Eliminate objects with the wrong number of component blobs
		if (componentBlobs.size() != 1 && componentBlobs.size() != 2 && componentBlobs.size() != 4) continue;

		// Average hue of all component blobs
		for (vector<Point> blob : componentBlobs){
			Rect enclosingRect = boundingRect(blob);
			Mat ROI = frameHUE(enclosingRect); // Dbg

			// Consider all points inside each component blob's bounding rectangle
			for (int x = enclosingRect.x; x < enclosingRect.x + enclosingRect.width; x += 1){
				for (int y = enclosingRect.y; y < enclosingRect.y + enclosingRect.height; y += 1){

					// Test returns positive value if point is inside contour
					int insideOutside = pointPolygonTest(blob, Point(x, y), false);	
					
					// Sum hues inside contour while counting pixels processed
					if (insideOutside > 0) {	
						hueSum += frameHUE.at<uchar>(Point(x, y));
						pixCount++;
						joinedBlobArea++;
						xSum += x;
						ySum += y;
						//frameHUE.at<uchar>(Point(x, y)) = 255; // Dbg
						continue;
					}
					if (insideOutside == 0) frameHUE.at<uchar>(Point(x, y)) = 0; // Dbg
					// else frameHUE.at<uchar>(Point(x, y)) = 122; // Dbg
				}
			}
		}	// End of hue averaging loop

		// Calculate object's average hue and centroid
		Point2d centroid;
		if (pixCount != 0){
			avgHue = (float)hueSum / pixCount;
			centroid = Point2d{ (double)xSum / pixCount, (double)ySum / pixCount };
		}

		// Eliminate objects with high hue values
		const int maxAcceptableHueAvg = 50;
		if (avgHue > maxAcceptableHueAvg) continue;
		
		// Orientation angle of vector from rect center to vehicle centroid
		double vehicleOrientation = -1; // -1 is flag for no vehicle rec
		//circle(frameHUE, centroid, 2, Scalar(0), CV_FILLED);
		//circle(frameHUE, currentObject.center, 2, Scalar(122), CV_FILLED);

		// Calculate ratio of short side to long side of the object's min Area Rect
		float RR_width = rotatedRectWidth(currentObject);
		float RR_height = rotatedRectHeight(currentObject);
		float w_h_ratio = RR_width/RR_height;
		//cout << "width(short): " << RR_width << endl;
		//cout << "height(long) : " << RR_height << endl;
		cout << "width/height : ** " << w_h_ratio << endl;

		// Set limits for acceptable width/height ratios for recognition
		
		// Calculate ratio of a the joined blob area to the area of its min area rectangle (NOT WORKING CORRECTLY!)
		Size_<float> minAreaRectSize = currentObject.size;
		float blobAreaRatio = joinedBlobArea / (minAreaRectSize.height * minAreaRectSize.width);
		//cout << "area ratio: " << blobAreaRatio << endl;
		cout << "avg Hue: " << avgHue << endl;

		// Outcomes of identification
		if (componentBlobs.size() == 2 && w_h_ratio > 0.7 && w_h_ratio < 0.9 && avgHue <= 20) {		// Delorean Identified
			identifiedVehicle = Recognized::DeLorean;
			objectsRecognized = true;			// Temp, want to replace this
			float orientation = currentObject.angle;
			string DeLorean = "DeLorean";
			Scalar orange = Scalar(0, 120, 255);
			int font = FONT_HERSHEY_SIMPLEX;
			int fontSize = 1;
			int fontThickness = 3;
			putText(outputFrame, DeLorean, objectCenter, font, fontSize, orange, fontThickness);
			//arrowedLine(outputFrame, currentObject.center, centroid, Scalar(255, 0, 0));
			vehicleOrientation = vehicleOrientationAngle(currentObject, centroid);
			drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 20);// Scalar(0, 255, 255));

			geometry_msgs::Point delorean_point;
			delorean_point.x = centroid.x * RE_UPSAMPLING_FACTOR;
			delorean_point.y = centroid.y * RE_UPSAMPLING_FACTOR;
			delorean_point.z = vehicleOrientation;
			delorean_pub.publish(delorean_point);
			//cout << "DeLorean: " << "x = " << delorean_point.x << " y = " << delorean_point.y << " z = " << delorean_point.z << endl; // DBG
		}

		if(w_h_ratio > 0.4 && w_h_ratio < 0.6 && avgHue <= 12){		// Could be handle
			//Mat handleROI = frameHUE(objectRectangle);

			// Calculate the average hue value of the pixels inside the object above a threshold
			const int hueThresh = 30;
			int _hueSum = 0;
			int _xSum = 0;
			int _ySum = 0;
			int _pixCount = 0;

			for (int y = objectRectangle.y; y < objectRectangle.y + objectRectangle.height; y ++){
				// imshow("hue Handle DBG", frameHUE);
				for (int x = objectRectangle.x; x < objectRectangle.x + objectRectangle.width; x ++){
					int _hueVal = frameHUE.at<uchar>(y,x);

					// Test returns positive value if point is inside contour
					int insideOutside = pointPolygonTest(minAreaRectContour, Point(x, y), false);

					// If pixel is inside object and isn't orange'ish
					if(insideOutside > 0 && _hueVal > hueThresh){
						_hueSum += _hueVal;
						_xSum += x;
						_ySum += y;
						_pixCount++;
					}
				}
			}
			cout << "hueSum: " << _hueSum << " pixCount: " << _pixCount << endl;
			if (_pixCount > 40) {
				int _avgHue = _hueSum / _pixCount;
				Point _centroid = Point(_xSum/_pixCount, _ySum/_pixCount);
				if (_avgHue > 130){
					cout << "Average Handle Hue:" << _avgHue << endl;
					circle(outputFrame, _centroid, 5, Scalar(255,0,255), CV_FILLED);
					string Handle = "Handle";
					putText(outputFrame, Handle, _centroid, FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,255), 3);
					geometry_msgs::Point handle_point;
					handle_point.x = _centroid.x * RE_UPSAMPLING_FACTOR;
					handle_point.y = _centroid.y * RE_UPSAMPLING_FACTOR;
					handle_point.z = vehicleOrientation;
					handle_pub.publish(handle_point);
				}
			}
		}
		if (componentBlobs.size() == 1 && fabs(w_h_ratio - 0.125) < 0.8 && avgHue <= 12){	// Guide marker identified
			// Create a point along the central long axis of the marker
			double dist01 = Distance(vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y);
			double dist12 = Distance(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y);
			Point ptOnLongAxis;
			if (dist01 < dist12) ptOnLongAxis = Point((vertices[0].x + vertices[1].x)/2, (vertices[0].y + vertices[1].y)/2);
			else ptOnLongAxis = Point((vertices[1].x + vertices[2].x)/2, (vertices[1].y + vertices[2].y)/2);
			double guideStripOrientation = rayOrientationAngle(currentObject.center, ptOnLongAxis);
			geometry_msgs::Point guide_strip_point;
			guide_strip_point.x = currentObject.center.x;
			guide_strip_point.y = currentObject.center.y;
			guide_strip_point.z = guideStripOrientation;
			guide_strip_pub.publish(guide_strip_point);

		}
		if (componentBlobs.size() == 4 && w_h_ratio > 0.4 && w_h_ratio < 0.6 && avgHue >= 12) {	// Train Identified
			identifiedVehicle = Recognized::Train;
			objectsRecognized = true;			// Temp, want to replace this
			int orientation = currentObject.angle;
			string Train = "Train";
			Scalar yellow = Scalar(0, 255, 255);
			int font = FONT_HERSHEY_SIMPLEX;
			int fontSize = 1;
			int fontThickness = 3;
			putText(outputFrame, Train, objectCenter, font, fontSize, yellow, fontThickness);
			vehicleOrientation = vehicleOrientationAngle(currentObject, centroid);
			drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 50);//, Scalar(0, 255, 255));

			geometry_msgs::Point train_point;
			train_point.x = centroid.x * RE_UPSAMPLING_FACTOR;
			train_point.y = centroid.y * RE_UPSAMPLING_FACTOR;
			train_point.z = vehicleOrientation;
			train_pub.publish(train_point);
			//cout << "Train : " << "x = " << train_point.x << " y = " << train_point.y << " z = " << train_point.z << endl; // DBG

		}
		// else {
		// 	geometry_msgs::Point delorean_point;
		// 	delorean_point.x = 0;
		// 	delorean_point.y = 0;
		// 	delorean_point.z = 0;
		// 	delorean_pub.publish(delorean_point);

		// 	geometry_msgs::Point train_point;
		// 	train_point.x = 0;
		// 	train_point.y = 0;
		// 	train_point.z = 0;
		// 	train_pub.publish(train_point);
			//cout << "DeLorean: " << "x = " << delorean_point.x << " y = " << delorean_point.y << " z = " << delorean_point.z << endl; // DBG
			//cout << "Train : " << "x = " << train_point.x << " y = " << train_point.y << " z = " << train_point.z << endl; // DBG
		// }	
	}	// End of object recognition loop




	// Set waypoint if no objects were recognized
	if (!objectsRecognized){

		Point waypoint = Point(0, 0);
		bool waypointSet = false;
		int smallestDistance = -1;
		int closestObjectIdx;

		// Find nearest eligible object
		for (int i = 0; i < rotatedRects.size(); i++){
			Point objectCenter = rotatedRects[i].center;
			int distToCenter = Distance(objectCenter.x, objectCenter.y, camCenter.x, camCenter.y);
			if (rotatedRects[i].size.area() < 100) continue;	// Don't set waypoint to possible image noise
			if (i == 0) {
				smallestDistance = distToCenter;
				closestObjectIdx = i;
				waypointSet = true;
			}
			else if (distToCenter <= smallestDistance) {
				smallestDistance = distToCenter;
				closestObjectIdx = i;
				waypointSet = true;
			}

			
		}
		if (waypointSet ) waypoint = rotatedRects[closestObjectIdx].center;
		


		// Draw arrow to  waypoint on output img
		if (waypoint.x != 0 && waypoint.y != 0){
			//arrowedLine(outputFrame, camCenter, waypoint, Scalar(0, 0, smallestDistance), 6);
			//putText(outputFrame, "Waypoint", camCenter, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3);
		}
	}

	// Draw origin and recognition for visualization
	Scalar green(0, 255, 0);
	Scalar red(0, 0, 255);
	Scalar color = objectsRecognized ? green : red;
	//circle(outputFrame, camCenter, 5, color, CV_FILLED);			// Origin
	circle(outputFrame, camCenter, RECOGNITION_RADIUS, color, 3);	// Recognition Circle
	
	// Display Results
	imshow("Vehicle Detection Output", outputFrame);
	waitKey(1);

	//destroyAllWindows();
}	

int main(int argc, char* argv[]){
	ros::init(argc, argv, "recovery_vision");

 	ros::NodeHandle n;

 	// Create and initialize slider
 	namedWindow("Thresholding");
 	rec_viz_thresh_slider = 140;
	createTrackbar("Saturation Threshold: ", "Thresholding", &rec_viz_thresh_slider, 255);

	// Since we're subscribing to an image, use an image_transport::Subscriber
	image_transport::ImageTransport it(n);
 	image_transport::Subscriber cam_sub = it.subscribe("/forward_camera/image_color",\
		 1, imgCallback); // Needs to be changed to downward camera

 	ros::spin();

 	return 0;
}