/*
 * GraphicsHandler.cpp
 *
 *  Created on: Aug 9, 2016
 *      Author: mykyta_denysov
 */

#include "marker_recognition/GraphicsHandler.h"

GraphicsHandler::GraphicsHandler()
{
  const double axis_length = 100;
  axis_coordinates_ = cv::Mat::zeros(4, 3, CV_64FC1);
  axis_coordinates_.at<double>(1,0) = axis_length;
  axis_coordinates_.at<double>(2,1) = axis_length;
  axis_coordinates_.at<double>(3,2) = axis_length;
}

GraphicsHandler::~GraphicsHandler()
{
}

void GraphicsHandler::drawCoordinateAxis(cv::Mat RMat,cv::Mat tvec,cv::Mat Cam_matrix,cv::Mat distCoeffs, cv::Mat& camera_frame)
{
  std::vector<cv::Point2d> axis_projected;
  cv::projectPoints(axis_coordinates_, RMat, tvec, Cam_matrix, distCoeffs, axis_projected);
  // draw lines
  cv::line(camera_frame,axis_projected[0],axis_projected[1],cv::Scalar(255,0,0),4);
  cv::line(camera_frame,axis_projected[0],axis_projected[2],cv::Scalar(0,255,0),4);
  cv::line(camera_frame,axis_projected[0],axis_projected[3],cv::Scalar(0,0,255),4);

  //    cv::line(camera_frame,cv::Point(axis_projected.at<double>(0,0),axis_projected.at<double>(1,0)),cv::Point(axis_projected.at<double>(0,1),axis_projected.at<double>(1,1)),cv::Scalar(255,0,0));
  //    cv::line(camera_frame,cv::Point(axis_projected.at<double>(0,0),axis_projected.at<double>(1,0)),cv::Point(axis_projected.at<double>(0,2),axis_projected.at<double>(1,2)),cv::Scalar(0,255,0));
  //    cv::line(camera_frame,cv::Point(axis_projected.at<double>(0,0),axis_projected.at<double>(1,0)),cv::Point(axis_projected.at<double>(0,3),axis_projected.at<double>(1,3)),cv::Scalar(0,0,255));
}


