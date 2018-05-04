#ifndef BASIC_CALC_H
#define BASIC_CALC_H

#pragma once
#include "opencv2/opencv.hpp"

struct circ{
    cv::Point2f p;
    float r;
};

namespace Basic_Calc
{
//modulo operations
int mod(int a, int b);
int modDistance(int a, int b, int m);
//euclidian norm and metric
double normEucl(cv::Point2f v);
double metricEucl(cv::Point2f p, cv::Point2f q);
//angle calculations
double calcAngle(cv::Point2f o, cv::Point2f p, cv::Point2f q);
double calcAngleClockwise(cv::Point2f p, cv::Point2f q);
double calcAngleClockwise(cv::Point2f o, cv::Point2f p, cv::Point2f q);
//checks if two rectangles intersect each other
bool rectIntersection(cv::Rect rect_1, cv::Rect rect_2);
//checks if rectangle is within image
bool rectInsideImage(float scale, cv::Rect rect, cv::Mat* image);
//calculates surrounding rectangle for the two given ones
cv::Rect surroundingRect(cv::Rect rect_1, cv::Rect rect_2);

//calculates intersection point of line ab and circle c
cv::Point2f circleLineCut(cv::Point2f a, cv::Point2f b, circ c);

//rotation functions
cv::Point2f rotate(cv::Point2f p, float angle);
std::vector<cv::Point2f> rotatePoints(cv::Point2f zero,std::vector<cv::Point2f> v, float angle);
//calculates polar coordinates of p
cv::Point2f polarCoord(cv::Point2f p);

//different functions modelled for test purposes
float gauss1DPlateau(float x, float border, float sigma);
float gauss2DPlateau(cv::Point2f p, float sigma_x, float sigma_y, float x_size);
float gaussian2Deasy(cv::Point2f p, float sigma_1, float sigma_2);
float gaussConstPdf(cv::Point2f p, float sigma_x1, float sigma_x2, float sigma_y, float plateau);
float gaussRotationPlateau(cv::Point2f p, float sigma_r, float r_1, float r_2, float alpha, float beta, float exponent);
}

#endif // BASIC_CALC_H
