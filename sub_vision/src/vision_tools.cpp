#include "vision_tools.h"
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>


using namespace cv;
using namespace std;

double distCameraToObject(double actualLengthMM, double folcalLengthMM, double lengthInPixels){
	return (actualLengthMM * folcalLengthMM) / lengthInPixels;
}

void drawRotatedRect(cv::Mat& img, cv::RotatedRect RR, cv::Scalar color, int lineThickness = 1){
	Point2f vertices[4];
	RR.points(vertices);
	for (int i = 0; i < 4; i++) {
		line(img, vertices[i], vertices[(i + 1) % 4], color, lineThickness);
	}
}


double rotatedRectWidth(cv::RotatedRect RR){
	Point2f vertices[4];
	RR.points(vertices);
	float dist01 = Distance(vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y);
	float dist12 = Distance(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y);
	return dist01 <= dist12 ? (double)dist01 : (double)dist12;
}
void rotatedRectWidth_TEST(){
	RotatedRect RR1{ Point2f(100.0, 100.0), Size2f(40.5, 20.5), 45.0 };
	RotatedRect RR2{ Point2f(100.0, 100.0), Size2f(20.5, 40.5), 45.0 };
	Mat img{ 200, 200, CV_8UC1, Scalar(0) };
	drawRotatedRect(img, RR1, Scalar(255), 2);
	drawRotatedRect(img, RR2, Scalar(255), 2);
	std::cout << "Should be 20.5:	" << rotatedRectWidth(RR1) << endl;
	std::cout << "Should be 20.5:	" << rotatedRectWidth(RR2) << endl;
	imshow("rotatedRectWidth_TEST()", img);
	waitKey(0);
}


double vehicleOrientationAngle(RotatedRect minAreaRect, Point2d centroid){	
	Point2f vertices[4];
	minAreaRect.points(vertices);
	double distV0_V1 = Distance(vertices[0].x, vertices[0].y, vertices[1].x, vertices[1].y);
	double distV1_V2 = Distance(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y);

	 // Create a point along the long central axis
	Point2f pLong;
	if (distV0_V1 <= distV1_V2){
		float x = (vertices[0].x + vertices[1].x) / 2.0;
		float y = (vertices[0].y + vertices[1].y) / 2.0;
		pLong = Point2f{ x, y };
	}
	else{
		float x = (vertices[1].x + vertices[2].x) / 2.0;
		float y = (vertices[1].y + vertices[2].y) / 2.0;
		pLong = Point2f{ x, y };
	}

	// Calculate the angle of the following rays with the horizontal (right)
	double    angle_rectCenter_pLong1 = rayOrientationAngle(minAreaRect.center, pLong);
	double    angle_pLong_rectCenter2 = rayOrientationAngle(pLong, minAreaRect.center);
	double angle_centroid_rectCenter3 = rayOrientationAngle(centroid, minAreaRect.center);

	// Use the fact that  a line from the centroid to the rectCenter 
	// should point forward for either vehicle
	double absDiff_1_3 = fabs(angle_rectCenter_pLong1 - angle_centroid_rectCenter3);
	double absDiff_2_3 = fabs(angle_pLong_rectCenter2 - angle_centroid_rectCenter3);
	if (absDiff_1_3 <= absDiff_2_3) return angle_rectCenter_pLong1;
	else return angle_pLong_rectCenter2;

	//Point2d rectCenter = minAreaRect.center;
	//double rayToCentroidAngle = rayOrientationAngle(rectCenter, centroid);
	//double rectOrientationAngle = minAreaRect.angle;
	//if (minAreaRect.size.width < minAreaRect.size.height) {
	//	rectOrientationAngle = 90 + rectOrientationAngle;
	//}
	//// At this point rectOrientation is either corect or off by 180 degrees
	//double angleDiff = rayToCentroidAngle - rectOrientationAngle;
	//
	//// rectOrientationAngle is correct
	//if (fabs(angleDiff) > 90.0) {
	//	if (rectOrientationAngle < 0) rectOrientationAngle += 360.0; // Force positive angle
	//	return rectOrientationAngle;
	//}
	//
	//// rectOrientationAngle is off by 180 degrees
	//else{
	//	if (rectOrientationAngle >= 180) rectOrientationAngle -= 180;
	//	else rectOrientationAngle += 180;
	//	return rectOrientationAngle;
	//}


}
void vehicleOrientationAngle_TEST(){
	Point2f org = Point2f(250.0, 250.0);
	Size2f size = Size2f{ 100.0, 50.0 };
	RotatedRect RR = RotatedRect{org, size, 30 };
	Point2d vehicleCentroid = Point2d{ 255.0, 255.0 };
	Mat img = Mat{ 500, 500, CV_8UC1, Scalar(0) };
	circle(img, vehicleCentroid, 1, Scalar(255), CV_FILLED);
	circle(img, org, 1, Scalar(120), CV_FILLED);
	drawRotatedRect(img, RR, Scalar(255));
	double vehicleOrientation = vehicleOrientationAngle(RR, vehicleCentroid);
	drawArrowByAngle(img, org, vehicleOrientation, 50, Scalar(255));
}

void drawArrowByAngle(cv::Mat& img, cv::Point origin, double angleDeg, double length, Scalar& color){
	double deltaX = 0;
	double deltaY = 0;
	double angleRad = degToRad(angleDeg);
	deltaX = length * cos(angleRad);
	deltaY = length * -sin(angleRad);
	Point endPoint = Point((int)deltaX + origin.x, (int)deltaY + origin.y);
	arrowedLine(img, origin, endPoint, color, 3);
	
}
void drawArrowByAngle_TEST(){
	Mat img = Mat(600, 600, CV_8UC1, Scalar(0));
	Point origin = Point(300, 300);
	double angleDEG, length;
	Scalar white = Scalar(255);
	for (int i = 0; i < 16; i++){
		if (i == 0) {		// Initialize
			angleDEG = 0;
			length = 30;
		}
		drawArrowByAngle(img, origin, angleDEG, length, white);
		angleDEG += 22.5;
		length += 10;
	}					// Set breakpoint here to check for correct intermediate image
	imshow("Debug Image", img);
	waitKey(0);


}

double rayOrientationAngle(cv::Point2d p1, cv::Point2d p2){
	Point2d p3 = Point2d(p2.x, p1.y);
	double y = -(p2.y - p3.y);
	double x = p3.x - p1.x;
	if (x == 0 && y == 0){
		cout << "You must use DISTINCT points, domain error\n";
		return -1;
	}
	double resultRad = atan2(y, x);
	double resultDeg = (resultRad * 180) / CV_PI;
	if (resultDeg < 0) resultDeg += 360;		// Force positive angle in [0, 360)
	return resultDeg;
}
void rayOrientationAngle_TEST(){

	// rayOrientationAngle() test
	Point2d origin = Point2d(10.0, 10.0);
	cout << "Should be 0:	" << rayOrientationAngle(origin, Point2d(11.0, 10.0)) << endl;
	cout << "Should be 45:	" << rayOrientationAngle(origin, Point2d(11.0, 9.0)) << endl;
	cout << "Should be 90:	" << rayOrientationAngle(origin, Point2d(10.0, 9.0)) << endl;
	cout << "Should be 135:	" << rayOrientationAngle(origin, Point2d(9.0, 9.0)) << endl;
	cout << "Should be 180:	" << rayOrientationAngle(origin, Point2d(9.0, 10.0)) << endl;
	cout << "Should be 225:	" << rayOrientationAngle(origin, Point2d(9.0, 11.0)) << endl;
	cout << "Should be 270:	" << rayOrientationAngle(origin, Point2d(10.0, 11.0)) << endl;
	cout << "Should be 315:	" << rayOrientationAngle(origin, Point2d(11.0, 11.0)) << endl;
	cout << "Should be 29.36:	" << rayOrientationAngle(origin, Point2d(26.0, 1.0)) << endl;
}

MatND weightedHistogram(Mat& img){
	Mat result(256,1, CV_32F);
	result = 0.0;
	Point currentPixel;
	float pixelValue = 0;
	const float addedWeightMultiplier = .5;
	float addedWeight = 0;
	float pixelContribution = 0;
	for (int i = 0; i < img.rows; i++){
		for (int j = 0; j < img.cols; j++){
			currentPixel = Point(j, i);
			pixelValue = (float)img.at<uchar>(currentPixel);
			addedWeight = addedWeightMultiplier * pixelValue - 1;
			pixelContribution = 1.0 + addedWeight;
			result.at<float>(pixelValue) += pixelContribution;
		}
	}
	return result;

}

Mat hueDistanceToAvg(Mat& src){
	Mat result{ src.rows, src.cols, CV_8U };
	int hueSum = 0;
	int valCount = 0;
	for (int i = 0; i < src.rows; i++){
		for (int j = 0; j < src.cols; j += 10){	//Sample every 10th pixel
			hueSum += src.at<uchar>(i, j);
			valCount++;
		}
	}
	int avg = hueSum / valCount;

	for (int i = 0; i < src.rows; i++){
		for (int j = 0; j < src.cols; j++){
			result.at<uchar>(i, j) = abs(src.at<uchar>(i, j) - avg);
		}
	}
	return result;
}

int _countPointsInsideCircle(std::vector<cv::Point> &pointList, const cv::Point &center, double radius){
	int tally = 0;

	// Increment tally every time a point on the list is found inside the given circle
	for (Point testPoint : pointList){
		if (withinCircle(testPoint, center, radius)) tally++;
	}
	return tally;
}

bool withinCircle(const cv::Point &testPoint, const cv::Point &center, double radius){
	if (Distance(testPoint.x, testPoint.y, center.x, center.y) <= radius) return true;
	else return false;
}

Mat pointDensityMap(std::vector<cv::Point> &points, cv::Size &mapSize, double neighborhoodRadius){
	Mat result{ mapSize, CV_8U };
	Point currentMapPoint;
	for (int i = 0; i < result.rows; i++)
	{
		for (int j = 0; j < result.cols; j++)
		{
			currentMapPoint = Point(j, i);
			result.at<uchar>(currentMapPoint) = _countPointsInsideCircle(points, currentMapPoint, neighborhoodRadius);
		}
	}

	return result * 20; //DBG
}

double Distance(double dX0, double dY0, double dX1, double dY1){
	return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}
double radToDeg(double radians){
	return radians * 180 / CV_PI;
}
double degToRad(double degrees){
	return  degrees * CV_PI / 180;
}

int calcIdealThreshold(Mat histogram){
	assert(histogram.channels() == 1);
	assert(histogram.size() == Size(1, 256) || histogram.size() == Size(256, 1));
	map<int, int> maximaWithIdx; // index (key) also represents a pixel intensity value

	// Stores histogram maxima in a std::map
	for (int HUEvalue = 1; HUEvalue < 255; HUEvalue++){
		int lastCount = histogram.at<float>(HUEvalue - 1);
		int currentCount = histogram.at<float>(HUEvalue);
		int nextCount = histogram.at<float>(HUEvalue + 1);
		if (currentCount >= lastCount && currentCount >= nextCount){
			pair<int, int> locMaximum{ currentCount, HUEvalue };
			maximaWithIdx.insert(locMaximum);
		}
	}
	assert(maximaWithIdx.size() > 0);
	std::map<int, int>::iterator it = --maximaWithIdx.end(); // Points to last element
	int largestLocalMax = it->second;
	--it;													// Now points to 2nd to last element
	int secLargestLocalMax = it->second;

	// Find local minimum in between the two highest maxima
	int idealThreshold = 0;
	int lowerBound = (largestLocalMax < secLargestLocalMax) ? largestLocalMax + 1 : secLargestLocalMax + 1;
	int upperBound = (largestLocalMax < secLargestLocalMax) ? secLargestLocalMax : largestLocalMax;
	for (int HUEvalue = lowerBound; HUEvalue < upperBound; HUEvalue++){
		int lastCount = histogram.at<float>(HUEvalue - 1);
		int currentCount = histogram.at<float>(HUEvalue);
		int nextCount = histogram.at<float>(HUEvalue + 1);
		if (currentCount <= lastCount && currentCount <= nextCount){
			idealThreshold = HUEvalue;
		}
	}
	assert(idealThreshold != 0);


	return idealThreshold;
}

Mat filterGray(Mat srcColor, int graynessThreshold){

	/*
	Function takes in a 3 channel image and returns a single channel binary image of the same size.
	Pixels which are close to black, white or a shade of gray will be floored while more colorful pixels
	will be set to the max value. "graynessThreshold" is the cutoff standard deviation for the three pixel components;
	pixels with a higher standard deviation than "graynessThreshold" will be set to the max value.
	- David Santiago Soto
	*/
	const int rows = srcColor.rows;
	const int cols = srcColor.cols;

	Mat result = Mat::zeros(rows, cols, CV_8U);


	for (int x = 0; x < rows; x++){
		for (int y = 0; y < cols; y++){

			float BGR_average, BGR_sum, BGR_stdDev = 0;

			Vec3b BGR = srcColor.at<Vec3b>(x, y);
			int blue = BGR[0];
			int green = BGR[1];
			int red = BGR[2];

			BGR_sum = red + green + blue;
			BGR_average = BGR_sum / 3;


			float blueDev = abs(blue - BGR_average);
			float greenDev = abs(green - BGR_average);
			float redDev = abs(red - BGR_average);

			BGR_stdDev = (redDev + greenDev + blueDev) / 3;

			if (BGR_stdDev < graynessThreshold) result.at<uchar>(x, y) = 0;
			else  result.at<uchar>(x, y) = 255;

		}
	}

	return result;
}