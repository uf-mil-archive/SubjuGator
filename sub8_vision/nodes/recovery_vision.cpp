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

#define PI           3.14159265358979323846

using namespace std;
using namespace cv;


class ImgConverter{
	ros::NodeHandle nh;

public:
	//cv_bridge::CvImagePtr img_ptr(new cv_bridge::CvImage());
	cv_bridge::CvImagePtr img_ptr;
	void imageCb(const sensor_msgs::ImageConstPtr& msg){
       try
       {
         img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
	}

};

int main(int argc, char* argv[]){


	// Constants
	const int SKIP = 0;						// Number of frames skipped after every retrieved and processed frame
	const double DOWNSAMPLING_FACTOR = .5;	// Downsampling scale factor
	const int RECOGNITION_RADIUS = 125;		// Only recognizes objects within this distance from origin

 	ros::init(argc, argv, "recovery_vision");

 	ros::NodeHandle nh;

	// Image containers
	Mat currentFrame, frameHSV, frameHUE, blobExtractionImg, outputFrame;

	ros::Publisher delorean_pub = nh.advertise<geometry_msgs::Point>("delorean", 1000);
	ros::Publisher train_pub = nh.advertise<geometry_msgs::Point>("train", 1000);
	//ros::Publisher tracks_pub = nh.Publisher<Geometry_msgs::Point>("tracks", 1000)

	// Named Windows	
	namedWindow("Vehicle Detection Output");
	namedWindow("DBG");

	// Enumeration
	enum class Recognized
	{
		None, DeLorean, Train
	};


	// // Open video file
	// VideoCapture vid("C:\\Users\\Santiago\\Documents\\Visual Studio 2013\\Projects\\OpenCV_test\\OpenCV_test\\Delorean Pics\\Underwater Test Video(3).mp4");
	// if (!vid.isOpened())
	// {
	// 	cout << "Cannot open the video file" << endl;
	// 	return -1;
	// }

	// Subscribe to camera feed
	ImgConverter ic;
	ros::Subscriber cam_sub = nh.subscribe("/forward_camera/image_rect_color/theora",\
		 1, &ImgConverter::imageCb, &ic);	


	// Start of the main program loop
	int frameCount = 0;
	while (true && ros::ok()){


		// // Grab a new frame
		// bool grabSuccess = vid.grab();
		// if (!grabSuccess) //if not successful, break loop
		// {
		// 	cout << "Cannot grab the frame from video file" << endl;
		// 	break;
		// }



		
		// if (frameCount % (SKIP + 1) != 0) continue;		// Allows us to skip frames

		// // Retrieve frame
		// bool retrieveSuccess = vid.retrieve(currentFrame); // retrieve video frame
		// if (!retrieveSuccess) //if not successful, exit main loop
		// {
		// 	cout << "Cannot retrieve the grabbed video frame" << endl;
		// 	break;
		// }

		// Get current frame
		// sensor_msgs::ImageConstPtr& imgPtr = // ?? I dont know what to set this to
		// sensor_msgs::CameraInfo& camInfo = // ?? I dont know what to set this to
		// cv_bridge::CvImagePtr cvimage = cv_bridge::toCvCopy(imgPtr, sensor_msgs::image_encodings::BGR8);
		// subjugator::ImageSource::Image image(cvimage->image, *cam_info);
		// currentFrame = img.image;
		currentFrame = ic.img_ptr->image;

		// Set the center of the downsampled frame as the frame origin
		const Point camCenter = Point(currentFrame.cols / 4, currentFrame.rows / 4);

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

		// Containers for hue averaging and centroid calculation
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
				drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 20);// Scalar(0, 255, 255));

				geometry_msgs::Point::Ptr delorean_point(new geometry_msgs::Point);
				delorean_point->x = centroid.x;
				delorean_point->y = centroid.y;
				delorean_point->z = vehicleOrientation;
				delorean_pub.publish(delorean_point);
				
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
				drawArrowByAngle(outputFrame, objectCenter, vehicleOrientation, 50);//, Scalar(0, 255, 255));

				geometry_msgs::Point::Ptr train_point(new geometry_msgs::Point);
				train_point->x = centroid.x;
				train_point->y = centroid.y;
				train_point->z = vehicleOrientation;
				train_pub.publish(train_point);
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
				putText(outputFrame, "Waypoint", camCenter, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3);
			}
		}


		// Draw origin and recognition for visualization
		Scalar green(0, 255, 0);
		Scalar red(0, 0, 255);
		Scalar color = objectsRecognized ? green : red;
		circle(outputFrame, camCenter, 5, color, CV_FILLED);			// Origin
		circle(outputFrame, camCenter, RECOGNITION_RADIUS, color, 3);	// Recognition Circle
					
		

		cout << "Frame: " << frameCount;
		
		// Display Results
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

		frameCount++;

	}	// End of While Loop

	cv::waitKey(0);
	return 0;
}	// End of main()
