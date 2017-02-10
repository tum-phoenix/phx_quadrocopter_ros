/*
 * SolvePNP.h
 *
 *  Created on: Aug 6, 2016
 *      Author: mykyta_denysov
 */

#ifndef SRC_ORBPOSEESTIMATIONSELF_SOLVEPNP_H_
#define SRC_ORBPOSEESTIMATIONSELF_SOLVEPNP_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string.h>
#include <fstream>

// PnP correspondency solver for pose estimation
class PnPSolver{
public:
  PnPSolver(cv::Mat Cam_matrix = cv::Mat::zeros(3,3,CV_64FC1),cv::Mat distCoeffs=cv::Mat::zeros(5,1,CV_64FC1)):Cam_matrix_(Cam_matrix),distCoeffs_(distCoeffs) {}

  ~PnPSolver() {}

  bool estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
                           const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
                           int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
                           float reprojectionError, double confidence );    // Ransac parameters

  // todo: check path variable
  void loadCSVdata (char *path);

  cv::Mat gettvec() {return tvec_;}

  cv::Mat getRMat() {return RMat_;}

  cv::Mat getCameraMatrix() {return Cam_matrix_;}

  cv::Mat getDistCoeffs() {return distCoeffs_;}

private:
  // camera parameters
  cv::Mat Cam_matrix_;
  cv::Mat distCoeffs_;

  // location parameters
  cv::Mat tvec_;
  cv::Mat RMat_;

};

#endif /* SRC_ORBPOSEESTIMATIONSELF_SOLVEPNP_H_ */
