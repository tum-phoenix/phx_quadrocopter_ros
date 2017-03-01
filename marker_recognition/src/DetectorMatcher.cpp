/*
 * DetectorMatcher.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: mykyta_denysov
 */

// import OpenCV stuff
#include "marker_recognition/DetectorMatcher.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;


DetectorMatcher::~DetectorMatcher()
{}

// detect keypoints
void DetectorMatcher::computeKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
  detector_->detect(image, keypoints);
}

// extract descriptors
void DetectorMatcher::computeDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  extractor_->compute(image, keypoints, descriptors);
}

static bool ratio_pred (const std::vector<cv::DMatch>& matches)
{
  // if 2 NN has been identified
  if (matches.size() > 1)
  {
    // check distance ratio
    if (matches[0].distance / matches[1].distance > 0.7)
    {
      return true;
    }
    return false;
  }
  else
  { // does not have 2 neighbours
    return true;
  }
  return false;
}

// perform ratio test on match vector extracted from image, returns number of removed matches
int DetectorMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
{
  //  int removed = 0;
  //  // for all matches
  //  matches.erase( std::remove_if( matches.begin(), matches.end(), ratio_pred),matches.end() );
  //  return removed;

  int removed = 0;
  // for all matches
  for ( auto matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
      // check distance ratio
      if ((*matchIterator)[0].distance > 0.75*(*matchIterator)[1].distance)
      {
        matchIterator->clear(); // remove match
        removed++;
      }
    }
    else
    { // does not have 2 neighbours
      matchIterator->clear(); // remove match
      removed++;
    }
  }
  return removed;
}

// perform symmety test between to matches detected in two images, pushed back into third match vector
void DetectorMatcher::symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                                    const std::vector<std::vector<cv::DMatch> >& matches2,
                                    std::vector<cv::DMatch>& symMatches )
{

  // for all matches image 1 -> image 2
  for (auto matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
  {

    // ignore deleted matches
    if (matchIterator1->empty() || matchIterator1->size() < 2)
      continue;

    // for all matches image 2 -> image 1
    for (auto matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
    {
      // ignore deleted matches
      if (matchIterator2->empty() || matchIterator2->size() < 2)
        continue;

      // Match symmetry test
      if ((*matchIterator1)[0].queryIdx ==
          (*matchIterator2)[0].trainIdx &&
          (*matchIterator2)[0].queryIdx ==
          (*matchIterator1)[0].trainIdx)
      {
        // add symmetrical match
        symMatches.push_back(
              cv::DMatch((*matchIterator1)[0].queryIdx,
              (*matchIterator1)[0].trainIdx,
            (*matchIterator1)[0].distance));
        break; // next match in image 1 -> image 2
      }
    }
  }
}

void DetectorMatcher::DistanceTest (std::vector<cv::DMatch>& matches_in)
{
  int dist_max = 0;

  for (auto matchIterator = matches_in.begin(); matchIterator != matches_in.end(); ++matchIterator)
  {
    int dist = matchIterator->distance;

    if (dist>dist_max)
    {
      dist_max = dist;
    }
  }

  std::vector<cv::DMatch> temp_matches = matches_in;
  matches_in.clear();

  for (auto matchIterator = temp_matches.begin(); matchIterator != temp_matches.end(); ++matchIterator)
  {
    if (matchIterator->distance < dist_max*0.9)
    {
      matches_in.push_back(*matchIterator);
    }
  }
}

// match using ratio and symmetry test
void DetectorMatcher::robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                   std::vector<cv::KeyPoint>& keypoints_frame, const cv::Mat& descriptors_model )
{
  // 1a. Detection of the ORB features
  computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches12, matches21;

  // 2a. From image 1 to image 2
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

  // 2b. From image 2 to image 1
  matcher_->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours

  // 3. Remove matches for which NN ratio is > than threshold
  // clean image 1 -> image 2 matches
  ratioTest(matches12);
  // clean image 2 -> image 1 matches
  ratioTest(matches21);

  // 4. Remove non-symmetrical matches
  symmetryTest(matches12, matches21, good_matches);

  // 5. Perform distance test
  DistanceTest(good_matches);

}

// match using ratio match only
void DetectorMatcher::fastRobustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                       std::vector<cv::KeyPoint>& frame_keypoints,
                                       const cv::Mat& descriptors_model )
{
  // 1a. Detection of the ORB features
  computeKeyPoints(frame, frame_keypoints);

  //DEBUG::draw Keypoints
  drawDetectedKeypoints(frame, frame_keypoints);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  computeDescriptors(frame, frame_keypoints, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches;
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 50);

  // 3. Remove matches for which NN ratio is > than threshold
  ratioTest(matches);

  // 4. Fill good matches container
  for (auto matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
  {
    if (!matchIterator->empty())
      good_matches.push_back((*matchIterator)[0]);
  }

  // 5. perform distance test
  DistanceTest(good_matches);
}

void DetectorMatcher::drawDetectedKeypoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints)
{
  //    cv::Mat test_img = image;
  cv::Mat test_img  = cv::Mat::zeros(image.rows,image.cols,CV_8UC3);
  test_img = test_img + image;
  for (auto it = keypoints.begin(); it!= keypoints.end(); ++it)
  {
    cv::circle(test_img, it->pt, 4, cv::Scalar(255,0,0),3);
  }
  cv::namedWindow("Keypoints", CV_WINDOW_NORMAL);
  cv::imshow("Keypoints",test_img);
  return;
}
//todo: add Kalman filter
