/*
 * DetectorMatcher.h
 *
 *  Created on: Jul 13, 2016
 *      Author: mykyta_denysov
 */

#ifndef SRC_ORBPOSEESTIMATIONSELF_DETECTORMATCHER_H_
#define SRC_ORBPOSEESTIMATIONSELF_DETECTORMATCHER_H_

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>

class DetectorMatcher{
public:
  /**
     * Generate ORB detector and Brute-Force matcher
   */
  DetectorMatcher()
  {
    // ORB matcher for now
    detector_ = cv::ORB::create(500,1.1f,30,40,0,3,0,31,40);
    extractor_ = cv::ORB::create();

    // BruteForce matcher with Norm Hamming only for now
    // todo: switch to KNN matcher?
    matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_HAMMING2, false);

  }
  /**
     * Destructor stub
     */
  virtual ~DetectorMatcher();
  /**
     * Compute keypoints of an image
     * @param image - Image
     * @param keypoints - Keypoint ouput vector
     */
  void computeKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
  /**
     * Compute the descriptors of an image given its keypoints
     * @param image - Image
     * @param keypoints - Keypoints
     * @param descriptors - Descriptor output
     */
  void computeDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
  /**
     * Perform ratio test
     * @param matches
     * @return
     */
  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches);
  // perform symmetry test
  /**
     * Perform symmetry test
     * @param matches1
     * @param matches2
     * @param symMatches
     */
  void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
                    const std::vector<std::vector<cv::DMatch> >& matches2,
                    std::vector<cv::DMatch>& symMatches);

  // Match feature points using ratio and symmetry test
  void robustMatch(const cv::Mat& frame,
                   std::vector<cv::DMatch>& good_matches,
                   std::vector<cv::KeyPoint>& keypoints_frame,
                   const cv::Mat& descriptors_model);

  // Match feature points using ratio test only
  void fastRobustMatch(const cv::Mat& frame,
                       std::vector<cv::DMatch>& good_matches,
                       std::vector<cv::KeyPoint>& frame_keypoints,
                       const cv::Mat& descriptors_model);

  void DistanceTest(std::vector<cv::DMatch>& matches_in);

  void drawDetectedKeypoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);

private:
  // pointer to the feature point detector object
  cv::Ptr<cv::FeatureDetector> detector_;
  // pointer to the feature descriptor extractor object
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  // pointer to the matcher object
  cv::Ptr<cv::DescriptorMatcher> matcher_;
};



#endif /* SRC_ORBPOSEESTIMATIONSELF_DETECTORMATCHER_H_ */
