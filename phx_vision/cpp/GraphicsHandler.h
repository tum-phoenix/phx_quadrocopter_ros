/*
 * GraphicsHandler.h
 *
 *  Created on: Aug 9, 2016
 *      Author: mykyta_denysov
 */

#ifndef SRC_ORBPOSEESTIMATIONSELF_GRAPHICSHANDLER_H_
#define SRC_ORBPOSEESTIMATIONSELF_GRAPHICSHANDLER_H_

#include <opencv2/opencv.hpp>

class GraphicsHandler{
public:
	GraphicsHandler();
	~GraphicsHandler();
	cv::Mat drawCoordinateAxis(cv::Mat RMat,cv::Mat tvec,cv::Mat Cam_matrix,cv::Mat distCoeffs, cv::Mat camera_frame);
public:
	cv::Mat axis_coordinates_;
};



#endif /* SRC_ORBPOSEESTIMATIONSELF_GRAPHICSHANDLER_H_ */
