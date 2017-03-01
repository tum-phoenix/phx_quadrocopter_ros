/*
 * PnPSolver.cpp
 *
 *  Created on: Aug 6, 2016
 *      Author: mykyta_denysov
 */
#include "marker_recognition/PnPSolver.h"

bool PnPSolver::estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
                                   const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
                                   int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
                                   float reprojectionError, double confidence )    // Ransac parameters
{
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector

  bool useExtrinsicGuess = false;

  bool success = cv::solvePnPRansac( list_points3d, list_points2d, Cam_matrix_, distCoeffs_, rvec, tvec,
                                     useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                                     inliers, flags );

  Rodrigues(rvec,RMat_);      // converts Rotation Vector to Matrix
  tvec_ = tvec;

  return success;
}

void PnPSolver::loadCSVdata (char *path)
{
  // load camera paramters from csv
  std::ifstream csvstream (path,std::ifstream::in);

  // create and fill temporary matrices
  Cam_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  distCoeffs_ = cv::Mat::zeros(1, 5, CV_64FC1);

  std::vector<std::string> readout;
  std::string temp;
  while(csvstream.good())
  {
    getline(csvstream, temp, ',');
    readout.push_back(temp);
  }

  Cam_matrix_.at<double>(0, 0) = std::atof(readout[0].c_str());       //      [ fx   0  cx ]
  Cam_matrix_.at<double>(1, 1) = std::atof(readout[3].c_str());       //      [  0  fy  cy ]
  Cam_matrix_.at<double>(0, 2) = std::atof(readout[2].c_str());       //      [  0   0   1 ]
  Cam_matrix_.at<double>(1, 2) = std::atof(readout[4].c_str());
  Cam_matrix_.at<double>(2, 2) = 1.;

  distCoeffs_.at<double>(0, 0) = std::atof(readout[7].c_str());
  distCoeffs_.at<double>(0, 1) = std::atof(readout[8].c_str());
  distCoeffs_.at<double>(0, 2) = std::atof(readout[9].c_str());
  distCoeffs_.at<double>(0, 3) = std::atof(readout[10].c_str());
  distCoeffs_.at<double>(0, 4) = std::atof(readout[11].c_str());
}
