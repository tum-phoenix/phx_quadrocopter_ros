/*
 * GraphicsHandler.cpp
 *
 *  Created on: Aug 9, 2016
 *      Author: mykyta_denysov
 */

#include "MainRunning.h"
#include <opencv2/opencv.hpp>

GraphicsHandler::GraphicsHandler()
{
	cv::Mat axis_coordinates_= 	cv::Mat::zeros(3, 4, CV_64FC1);
	axis_coordinates_.at<double>(0,1) = 1;
	axis_coordinates_.at<double>(1,2) = 1;
	axis_coordinates_.at<double>(2,3) = 1;
}

GraphicsHandler::~GraphicsHandler()
{
    // auto-generated destructor stub
}

cv::Mat GraphicsHandler::drawCoordinateAxis(cv::Mat RMat,cv::Mat tvec,cv::Mat Cam_matrix,cv::Mat distCoeffs, cv::Mat camera_frame)
{
	cv::Mat frame_overlay;
	cv::projectPoints(axis_coordinates_, RMat, tvec, distCoeffs, Cam_matrix, frame_overlay);
	return frame_overlay;
}


