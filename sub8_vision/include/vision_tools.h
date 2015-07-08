#ifndef VISION_TOOLS_H
#define VISION_TOOLS_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int calcIdealThreshold(cv::Mat histogram);
cv::Mat filterGray(cv::Mat srcColor, int graynessThreshold);

double Distance(double dX0, double dY0, double dX1, double dY1);
double radToDeg(double radians);
double degToRad(double degrees);

double distCameraToObject(double actualLengthMM, double folcalLengthMM, double lengthInPixels);

void drawRotatedRect(cv::Mat& img, cv::RotatedRect RR, cv::Scalar color, int lineThickness);

// Returns the width (smallest dimension) of a RotatedRect
double rotatedRectWidth(cv::RotatedRect RR);
void rotatedRectWidth_TEST();

// Returns the orientation angle of a ray starting at p1 and passing through p2
// Pointing right -> 0	degrees	Pointing up -> 90 degrees
double rayOrientationAngle(cv::Point2d p1, cv::Point2d p2);
void rayOrientationAngle_TEST();

// BTTF vehicle forward orientation angle (always positive)
double vehicleOrientationAngle(cv::RotatedRect minAreaRect, cv::Point2d centroid);
void vehicleOrientationAngle_TEST();

// Draws an arrow defined by a point of origin, an angle, and a length
void drawArrowByAngle(cv::Mat& img, cv::Point origin, double angleDeg, double length);//, cv::Scalar& color);
void drawArrowByAngle_TEST();

// Function will create a local point density map illustrating the amount of points
// in a given set which are within a set distance of each point in the density map
cv::Mat pointDensityMap(std::vector<cv::Point> &points, cv::Size &mapSize, double neighborhoodRadius);


// Function will determine if a point is inside a given circle
bool withinCircle(const cv::Point&, const cv::Point &center, double radius);

// Hue distance from average
cv::Mat hueDistanceToAvg(cv::Mat& src);

// Weighted Histogram
// Adds a weighted constant to the contribution of each pixel in the matrix based on the value of said pixel
cv::MatND weightedHistogram(cv::Mat& img);

#endif