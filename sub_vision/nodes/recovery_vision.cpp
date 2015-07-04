#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_tools.h"
#include "BTTF_Vehicle.h"
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Point.h"

#define PI           3.14159265358979323846

using namespace std;
using namespace cv;

//class DS_Blob{
//public:
//
//	vector<Point> contour;
//	double area = 0;
//	Point2d centroid = Point2d(0, 0);
//
/ls/	// Constructor
//	DS_Blob(vector<Point>* contour, double area, Point2d centroid){
//		this->contour = *contour;
//		this->area = area;
//		this->centroid = centroid;
//	}
//	
//	// Copy constructor
//	DS_Blob(const DS_Blob &source){
//		this->contour = source.contour;
//		this->area = source.area;
//		this->centroid = source.centroid;
//	}
//	
//	DS_Blob& operator= (const DS_Blob &source){
//		this->contour = source.contour;
//		this->area = source.area;
//		this->centroid = source.centroid;
//	
//		return *this;
//	
//	}
//};
//int main1(int argc, char* argv[]){
//	// Open video file
//	VideoCapture vid("C:\\Users\\Santiago\\Documents\\Visual Studio 2013\\Projects\\OpenCV_test\\OpenCV_test\\Delorean Pics\\Underwater Test Video(3).mp4");
//	if (!vid.isOpened())
//	{
//		cout << "Cannot open the video file" << endl;
//		return -1;
//	}
//	Mat img, imgHue;
//	vector<Mat> separatedHSV;
//
//	vid.read(img);
//
//	// Converting to HSV colorspace and isolating Hue channel
//	cvtColor(img, img, CV_BGR2HSV);
//	split(img, separatedHSV);
//	imgHue = separatedHSV[0].clone();
//
//	// Blurring
//	Size kernelSize = Size(3, 3);
//	GaussianBlur(imgHue, imgHue, kernelSize, 0);
//
//
//
//
//	return 0;
//}

int main1(){
	rotatedRectWidth_TEST();//vehicleOrientationAngle_TEST();//rotatedRectWidth_TEST();
	return 0;
}

int main(int argc, char* argv[]){


	// Constants
	const int SKIP = 0;						// Number of frames skipped after every retrieved and processed frame
	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const int RECOGNITION_RADIUS = 125;		// Only recognizes objects within this distance from origin

 	ros::init(argc, argv, "recovery_vision");

 	ros::NodeHandle nh;

	// Image containers
	Mat currentFrame, frameHSV, frameHUE, blobExtractionImg, outputFrame;

	ros::Publisher delorean_pub = nh.Publisher<geometry_msgs::Point>("delorean", 1000)
	ros::Publisher train_pub = nh.Publisher<geometry_msgs::Point>("train", 1000)
	//ros::Publisher tracks_pub = nh.Publisher<Geometry_msgs::Point>("tracks", 1000)

	// Named Windows	
	namedWindow("Vehicle Detection Output");
	namedWindow("DBG");

	// Enumeration
	enum class Recognized
	{
		None, DeLorean, Train
	};


	// Open video file
	VideoCapture vid("C:\\Users\\Santiago\\Documents\\Visual Studio 2013\\Projects\\OpenCV_test\\OpenCV_test\\Delorean Pics\\Underwater Test Video(3).mp4");
	if (!vid.isOpened())
	{
		cout << "Cannot open the video file" << endl;
		return -1;
	}


	// Set the center of the downsampled frame as the frame origin
	const Point camOrigin = Point(vid.get(CV_CAP_PROP_FRAME_WIDTH) / 4, vid.get(CV_CAP_PROP_FRAME_HEIGHT) / 4);


	// Start of the main program loop
	int frameCount = 0;
	while (true && ros::ok()){


		// Grab a new frame
		bool grabSuccess = vid.grab();
		if (!grabSuccess) //if not successful, break loop
		{
			cout << "Cannot grab the frame from video file" << endl;
			break;
		}

		
		if (frameCount % (SKIP + 1) == 0){		// Allows us to skip frames

			// Retrieve frame
			bool retrieveSuccess = vid.retrieve(currentFrame); // retrieve video frame
			if (!retrieveSuccess) //if not successful, exit main loop
			{
				cout << "Cannot retrieve the grabbed video frame" << endl;
				break;
			}

			// Downsampling and blurring
			resize(currentFrame, outputFrame, Size(0, 0), DOWNSAMPLING_FACTOR, DOWNSAMPLING_FACTOR, INTER_NEAREST);
			Size kernelSize = Size(3, 3);
			GaussianBlur(outputFrame, outputFrame, kernelSize, 0);

			// Containers for color conversion
			frameHSV = Mat::zeros(outputFrame.size(), CV_8U);
			vector<Mat> separtedHSV(3);
			vector<vector<Point>> HSVcontours;

			// Converting to HSV colorspace and isolating Hue channel
			cvtColor(outputFrame, frameHSV, CV_BGR2HSV);
			split(frameHSV, separtedHSV);
			frameHUE = separtedHSV[0].clone();

			// Applying flood fill to Hue channel
			blobExtractionImg = frameHUE.clone();
			floodFill(blobExtractionImg, Point(0, 0), Scalar(0), 0, 3, 3, 8);
			
			// Containers for blob extraction
			vector<vector<Point>> floodFillContours, joinedFFContours;

			// Blob extraction
			findContours(blobExtractionImg, floodFillContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);	
			
			// Joining Blobs that are close together
			blobExtractionImg = Mat::zeros(outputFrame.size(), CV_8U);
			double distThresh = 50;
			drawContours(blobExtractionImg, floodFillContours, -1, Scalar(255), 2);		// Draw blob mask image
			for (int i = 0; i < floodFillContours.size(); i++){
				for (int j = 0; j < floodFillContours.size(); j++){

					// Connect blob masks with lines on image
					Point p1 = floodFillContours[i][0];
					Point p2 = floodFillContours[j][0];
					if (Distance(p1.x, p1.y, p2.x, p2.y) < distThresh) line(blobExtractionImg, p1, p2, Scalar(255), 2);
				}
			}

			// Extract object contours from joined blobs
			findContours(blobExtractionImg, joinedFFContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			drawContours(blobExtractionImg, joinedFFContours, -1, Scalar(122), 1);
			
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

			// Containers for hue averaging
			int avgHue = 0;
			int pixCount = 0;
			int hueSum = 0;
			int xSum = 0;
			int ySum = 0;


			// Object recognition loop
			Recognized identifiedVehicle = Recognized::None; //  0 -> No vehicle identified
			bool objectsRecognized = false;			// Temp, want to replace this
			for (RotatedRect currentObject : rotatedRects){
				

				// Object properties
				Point2f objectCenter = currentObject.center;
				Rect objectRectangle = currentObject.boundingRect();
				float distanceToOrigin = Distance(camOrigin.x, camOrigin.y, objectCenter.x, objectCenter.y);


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



				// Reset hue averaging counters
				hueSum = pixCount = xSum = ySum = 0;

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
								xSum += x;
								ySum += y;
								frameHUE.at<uchar>(Point(x, y)) = 255; // Dbg
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
				
				// Orientation angle of vector from rect center to vehicle centroid
				double vehicleOrientation = -1; // -1 is flag for no vehicle rec
				circle(frameHUE, centroid, 2, Scalar(0), CV_FILLED);
				circle(frameHUE, currentObject.center, 2, Scalar(122), CV_FILLED);
				

				// Outcomes of identification
				if (avgHue > 21 && avgHue < 39) {		// Delorean Identified
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
					drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 20, Scalar(0, 255, 255));
					delorean_point = geometry_msgs::Point(x = centroid.x
														  y = centroid.y
														  z = vehicleOrientation
														)
					delorean_pub.publish(delorean_point)
					
				}
				else if (avgHue > 41 && avgHue < 59) {	// Train Identified
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
					drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 50, Scalar(0, 255, 255));
					train_point = geometry_msgs::Point(x = centroid.x
														  y = centroid.y
														  z = vehicleOrientation
														)
					train_pub.publish(train_point)
				}

				
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
					int distToCenter = Distance(objectCenter.x, objectCenter.y, camOrigin.x, camOrigin.y);
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
					arrowedLine(outputFrame, camOrigin, waypoint, Scalar(0, 0, smallestDistance), 6);
					putText(outputFrame, "Waypoint", camOrigin, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3);
				}
			}


			// Draw origin and recognition for visualization
			Scalar green(0, 255, 0);
			Scalar red(0, 0, 255);
			Scalar color = objectsRecognized ? green : red;
			circle(outputFrame, camOrigin, 5, color, CV_FILLED);			// Origin
			circle(outputFrame, camOrigin, RECOGNITION_RADIUS, color, 3);	// Recognition Circle
						
			//// Corner Detection (Harris)
			//Mat corners(frameHUE.size(), CV_32FC1, Scalar(0));
			//Mat corners_norm, corners_norm_scaled;
			//
			///// Detector parameters
			//int blockSize = 2;
			//int apertureSize = 9;
			//double k = 0.04;
			//
			///// Detecting corners
			//cornerHarris(frameHUE, corners, blockSize, apertureSize, k, BORDER_DEFAULT);
			//
			///// Normalizing
			//normalize(corners, corners_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
			//convertScaleAbs(corners_norm, corners_norm_scaled);
			//
			//
			//
			//// Initialize histogram parameters
			//int histSize = 256;    // bin size
			//float range[] = { 0, 256 };
			//const float *ranges[] = { range };
			//
			//// Calculate histogram
			//MatND hist;
			//calcHist(&corners_norm_scaled, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false);
			//
			//// Plot the histogram
			//int hist_w = 512; int hist_h = 400;
			//int bin_w = cvRound((double)hist_w / histSize);
			//
			//Mat histImage(hist_h, hist_w, CV_8UC1, Scalar(0, 0, 0));
			//
			//float normalizeMaxVal = histImage.rows;
			//normalize(hist, hist, 0, normalizeMaxVal, NORM_MINMAX, -1, Mat());
			//
			//for (int i = 1; i < histSize; i++)
			//{
			//	line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			//		Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
			//		Scalar(255, 0, 0), 2, 8, 0);
			//}
			//
			//// Calculate ideal corner response threshold
			//int histMaxIdx = 0;		// Idx of histograms mode
			//for (int idx = 0; idx < hist.rows; idx++){
			//	if (hist.at<float>(idx) == normalizeMaxVal) {
			//		histMaxIdx = idx;
			//		break;
			//	}
			//	if (idx == 255) { cout << "Error: could not find the histogram's mode"; return -1; }
			//}
			//int cornerResponseThresh = histMaxIdx + 10;
			//
			///// Drawing a circle around corners
			//vector<Point> cornersVec;
			//for (int j = 0; j < corners_norm_scaled.rows; j++)
			//{
			//	for (int i = 0; i < corners_norm_scaled.cols; i++)
			//	{
			//		if ((int)corners_norm_scaled.at<uchar>(j, i) > cornerResponseThresh)
			//		{
			//			Point corner = Point(i, j);
			//			cornersVec.push_back(corner);
			//			circle(outputFrame, corner, 5, Scalar(122), 1, 8, 0);
			//		}
			//	}
			//}
			//
			//vector<vector<int>> pairsOfPoints;
			//for (int i = 0; i < cornersVec.size(); i++){
			//	for (int j = 0; j < cornersVec.size(); j++){
			//		vector<int> pair{ i, j };
			//		pairsOfPoints.push_back(pair);
			//	}
			//}
			//for (int pairCount = 0; pairCount < pairsOfPoints.size(); pairCount++){
			//	Point first = cornersVec[pairsOfPoints[pairCount][0]];
			//	Point second = cornersVec[pairsOfPoints[pairCount][1]];
			//	if (Distance(first.x, first.y, second.x, second.y) < 50) line(outputFrame, first, second, Scalar(255, 255, 255));
			//}
			//
			//int idealThreshold = calcIdealThreshold(hist);
			//cout << "Ideal threshold: " << idealThreshold << endl;
			//
			//
			//
			//threshold(frameHUE, threshedframeHue, 70, 255, THRESH_BINARY_INV);
			//Mat silhouettes = threshedframeHue.clone();
			//findContours(threshedframeHue, HSVcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			//drawContours(outputFrame, HSVcontours, -1, Scalar(0, 0, 255), 1);
			//
			//circle(outputFrame, camOrigin, 10, Scalar(0, 0, 255), CV_FILLED);
			//
			//
			//const double areaThreshold = 100;
			//vector<Moments> blobMoments;
			//vector<DS_Blob> filteredBlobs;
			//
			//// Calculate contour moments for all blobs
			//for (int i = 0; i < HSVcontours.size(); i++){
			//	blobMoments.push_back(moments(HSVcontours[i]));
			//}
			//
			//// Store the contours, areas, and centroids of selected blobs that surpass the area threshold
			//for (int i = 0; i < HSVcontours.size(); i++){
			//	if (blobMoments[i].m00 > areaThreshold) {
			//		vector<Point> contour = HSVcontours[i];
			//		double area = blobMoments[i].m00;
			//		Point2d centroid = Point2d(blobMoments[i].m10 / blobMoments[i].m00, blobMoments[i].m01 / blobMoments[i].m00);
			//		DS_Blob areaThreshedBlob(&contour, area, centroid);
			//		filteredBlobs.push_back(areaThreshedBlob);
			//	}
			//}
			//
			//threshedframeHue = silhouettes.clone();
			//
			//vector<vector<double>> distances(filteredBlobs.size());
			//// distances[i][j] should represent the distance between the centroids of blobs[i] and blobs[j]
			//for (int i = 0; i < filteredBlobs.size(); i++){
			//	for (int j = 0; j < filteredBlobs.size(); j++){
			//		Point2d ci = filteredBlobs[i].centroid;
			//		Point2d cj = filteredBlobs[j].centroid;
			//		double distance = Distance(ci.x, ci.y, cj.x, cj.y);
			//		distances[i].push_back(distance);
			//		double distanceThreshold = 2 * sqrt(filteredBlobs[i].area + filteredBlobs[j].area);
			//		if (distance < distanceThreshold) {
			//			line(threshedframeHue, ci, cj, 255, 2);				// Join blobs in close proximity
			//		}
			//	}
			//}
			//
			//At this point blobs belonging to the same vehicle should have been joined
			//			
			//vector<vector<Point>> combinedBlobContours;
			//findContours(threshedframeHue, combinedBlobContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			//for (int i = 0; i < combinedBlobContours.size(); i++){
			//	if (combinedBlobContours[i].size() < 10) combinedBlobContours.erase(combinedBlobContours.begin() + i);
			//}
			//
			//
			//vector<RotatedRect> vehicleEnclosingRects(combinedBlobContours.size());
			//for (int i = 0; i < combinedBlobContours.size(); i++){
			//	vehicleEnclosingRects[i] = minAreaRect(combinedBlobContours[i]);
			//	Point2f rRectVertices[4];
			//	vehicleEnclosingRects[i].points(rRectVertices);
			//	for (int j = 0; j < 4; j++){
			//		if (j < 3){
			//			line(outputFrame, rRectVertices[j], rRectVertices[j + 1], Scalar(0, 255, 0), 3);
			//		}
			//		else{
			//			line(outputFrame, rRectVertices[j], rRectVertices[0], Scalar(0, 255, 0), 3);
			//		}
			//	}
			//}
			//
			//Point2d waypoint;
			//double distToWP;
			//for (int i = 0; i < vehicleEnclosingRects.size(); i++){
			//	Point2d origin = camOrigin;
			//	Point2d rectCenter = vehicleEnclosingRects[i].center;
			//	double rectArea = vehicleEnclosingRects[i].size.area();
			//	if (i == 0) {
			//		waypoint = rectCenter;
			//		distToWP = Distance(origin.x, origin.y, waypoint.x, waypoint.y);
			//	}
			//	else {
			//		if (Distance(origin.x, origin.y, rectCenter.x, rectCenter.y) < Distance(origin.x, origin.y, waypoint.x, waypoint.y) && rectArea > areaThreshold){
			//			waypoint = rectCenter;
			//			distToWP = Distance(origin.x, origin.y, waypoint.x, waypoint.y);
			//		}
			//	}
			//}
			//
			//// This arrow represents setting a waypoint to the location pointed to
			//arrowedLine(outputFrame, camOrigin, waypoint, Scalar(0, 0, 0), 3);

			cout << "Frame: " << frameCount << "	Time: " << vid.get(CV_CAP_PROP_POS_MSEC) / 1000 << " s\n";
			
			// Display Results
			//imshow("Debugging", blobExtractionImg);
			imshow("Vehicle Detection Output", outputFrame);
			imshow("DBG", frameHUE);
			
			// Release Memory
			//frameHSV.release();
			//silhouettes.release();
			//frameHUE.release();
			//outputFrame.release();
			//currentFrame.release();

			// Wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
			if (cv::waitKey(1) == 27)
			{
				cout << "ESC key was pressed by user" << endl;
				break;
			}

		}

		frameCount++;

	}	// End of While Loop

	cv::waitKey(0);
	return 0;
}	// End of main()
