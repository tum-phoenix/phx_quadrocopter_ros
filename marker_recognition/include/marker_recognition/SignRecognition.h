/*
 * MainRunning.h
 *
 *  Created on: Jul 25, 2016
 *      Author: mykyta_denysov
 */

/**
 * Basic recognition
 */

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <getopt.h>
#include <utility>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "DetectorMatcher.h"
#include "PnPSolver.h"
#include "GraphicsHandler.h"

class SignRecognition {
public:
  /**
   * Constructor, grabs all images with prefixes in argv from launch folder and detects them in webcam stream "0"
   * @param argc
   * @param argv
   */
  SignRecognition(std::vector<std::string> reference_images, std::string parameters, int method);
  /**
   * Desctructor stub, cleans up the calculation classes
   */
  virtual ~SignRecognition();
  void updateImage(cv::Mat& image_in);
  void matchContours(cv::Mat& image_in);
  void detectORB(cv::Mat& image_in);
  void MSER(cv::Mat& image_in);
  /**
   * Compute keypoints and descriptors for a vector of source images
   * @param images_source - Vector of cv::Mat source images
   * @param source_keypoints - Container for keypoints, map of integers to vectors of cv::KeyPoint, integers map keypoint vectors to their corresponding image in the source image vector
   * @param source_descriptors - Container for descriptors, map of integers to cv::Mat, integers map descriptors to their corresponding image in the source image vector
   */
  void describeImages(std::vector<cv::Mat>& images_source, std::map<int,std::vector<cv::KeyPoint> >& source_keypoints, std::map<int,cv::Mat>& source_descriptors);
  /**
   * match a provided set of keypoints and descriptors with the current frame, save as vector of 2d and 3d points
   * @param source_keypoints - Keypoints of the match image, vector of cv::KeyPoint
   * @param source_descriptors - Descriptors of the source image, cv::Mat
   * @param frame_image - Camera image frame, cv::Mat
   * @param points_3d - Matched 3d reference points, vector of cv::Point3f
   * @param points_2d - Matched 2d points in camera frame, vector of cv::Point2f
   */
  void matchImages(std::vector<cv::KeyPoint>& source_keypoints, cv::Mat& source_descriptors, cv::Mat& frame_image, std::vector<cv::Point3f>& points_3d, std::vector<cv::Point2f>& points_2d, cv::Mat& source_image);
private:
  /**
   * Calculation class for detection and matching of keypoints
   */
  DetectorMatcher my_detectormatcher_;
  /**
   * Calculation class for solving of PnP correspondence
   */
  PnPSolver my_pnpsolver_;
  /**
   * Post-Processing display class for visual output
   */
  GraphicsHandler my_graphicshandler_;
  /**
   * Descriptors of source images
   */
  std::map<int,cv::Mat> source_descriptors_;
  /**
   * Keypoints of source images
   */
  std::map<int,std::vector<cv::KeyPoint> > source_keypoints_;
  /**
     * Source images
     */
  std::vector<cv::Mat> images_source_;

  // match container
  std::map<int,cv::DMatch> good_matches_;

  // matched point containers
  std::vector<cv::Point3f> points_3d_;
  std::vector<cv::Point2f> points_2d_;

  // inlier container
  cv::Mat inliers_idx_;

  std::map<int,std::vector<cv::Point> > object_contours_;

  // PnP solver parameters
  int pnpMethod_ = cv::SOLVEPNP_ITERATIVE;
  int iterationscount_ = 1000;
  float reprojectionError_ = 2.0;
  double confidence_ = 0.95;

  // flags
  bool use_contours_;
  bool use_ORB_;
};
