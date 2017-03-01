/*
 * MainRunning.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: mykyta_denysov
 */

#include "marker_recognition/SignRecognition.h"

SignRecognition::SignRecognition(std::vector<std::string> reference_images, std::string parameters, int method):
  source_keypoints_(),
  source_descriptors_(),
  good_matches_(),
  points_3d_(),
  points_2d_()
{
  if (method == 0)
  {
    use_ORB_ = true;
    use_contours_ = false;
  }
  else if (method == 1)
  {
    use_contours_ = true;
    use_ORB_ = false;
  }
  else
  {
    ROS_INFO("Invalid method provided, using ORB.");
    use_ORB_ = true;
    use_contours_ = false;
  }

  my_pnpsolver_ = PnPSolver();
  char* csv_adress = new char[parameters.size()+1];
  std::copy(parameters.begin(),parameters.end(), csv_adress);
  csv_adress[parameters.size()] = '\0';
  my_pnpsolver_.loadCSVdata(csv_adress);
  my_graphicshandler_ = GraphicsHandler();

  //    KalmanFilter KF;             // instantiate Kalman Filter
  //    int nStates = 18;            // the number of states
  //    int nMeasurements = 6;       // the number of measured states
  //    int nInputs = 0;             // the number of control actions
  //    double dt = 0.125;           // time between measurements (1/FPS)

  //    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
  //    Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(Scalar(0));
  //    bool good_measurement = false;

  for (int i=0; i<reference_images.size(); ++i) {
    images_source_.push_back(cv::imread(reference_images[i]));
  }

  // get source image keypoints and descriptors
  if (use_ORB_)
  {
    describeImages(images_source_,source_keypoints_,source_descriptors_);
  }

  // DEBUG: draw keypoints of reference images
  //    int image_index = 0;
  //    for (auto it_image_kp = source_keypoints_.begin(); it_image_kp!=source_keypoints_.end(); ++it_image_kp)
  //    {
  //        my_detectormatcher_.drawDetectedKeypoints(images_source_[image_index],it_image_kp->second);
  //        image_index++;
  //    }

  // get contours of source images
  if (use_contours_)
  {
    for (int i=0; i<images_source_.size(); ++i)
    {
      // detect
      cv::RNG rng(12345);
      cv::Mat canny_output;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      int thresh = 100;

      // Detect edges using canny
      cv::Canny( images_source_[i], canny_output, thresh, thresh*2, 3 );
      // Find contours
      cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,cv::Point(0, 0) );

      // store contours and draw them (DEBUG)
      cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
      int max_area = 0;
      std::vector<cv::Point> max_contour;
      for( int j = 0; j< contours.size(); j++ )
      {
        if (max_area<cv::contourArea(contours[j]))
        {
          max_area = cv::contourArea(contours[j]);
          max_contour = contours[j];
          drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
          cv::drawContours( drawing, contours, j, cv::Scalar(255,0,0), 2, 8, hierarchy, 0, cv::Point() );
        }
      }
      cv::approxPolyDP(max_contour, object_contours_[i], 1.0, true);
      cv::namedWindow( "Contours detected in reference", CV_WINDOW_AUTOSIZE );
      cv::imshow( "Contours detected in reference", drawing );
      cv::waitKey(0);
    }
  }
}

void SignRecognition::updateImage(cv::Mat& image_in)
{
  if (image_in.empty())
  {
    ROS_ERROR("Received empty image update!");
    return ;
  }

  //    MSER(image_in);
  if (use_contours_)
    matchContours(image_in);
  if (use_ORB_)
    detectORB(image_in);
}

void SignRecognition::matchContours(cv::Mat& image_in)
{
  // detect
  cv::RNG rng(12345);
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  int thresh = 100;

  // Detect edges using canny
  cv::Canny( image_in, canny_output, thresh, thresh*2, 3 );
  // Find contours
  cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,cv::Point(0, 0) );

  // Draw contours
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
  double similarity = 0.0;
  double match_val  = 0.0;
  for( int i = 0; i< contours.size(); i++ )
  {
    cv::approxPolyDP(contours[i],contours[i],1.0,true);

    for (int j = 0; j<object_contours_.size(); j++)
    {
      match_val = cv::matchShapes(contours[i],object_contours_[j],CV_CONTOURS_MATCH_I3,1.0);
      if (similarity < match_val)
        similarity = match_val;
    }

    if (similarity>1000.0)
    {
      ROS_INFO_STREAM("Similarity is: "<<similarity);
      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    }
  }

  // Show in a window
  cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  cv::imshow( "Contours", drawing );
  cv::waitKey(1.0);
}

void SignRecognition::MSER(cv::Mat& image_in)
{
  //    cv::MSER ms;
  //    std::vector<std::vector<cv::Point>> regions;
  //    ms(box, regions, cv::Mat());
  //    for (int i = 0; i < regions.size(); i++)
  //    {
  //        cv::ellipse(box, cv::fitEllipse(regions[i]), cv::Scalar(255));
  //    }
  //    cv::imshow("mser", box);
  //    cv::waitKey(1);

  std::vector<std::vector<cv::Point>> contours;
  std::vector< cv::Rect> bboxes;
  cv::Ptr< cv::MSER> mser = cv::MSER::create(21, (int)(0.00002*image_in.cols*image_in.rows), (int)(0.05*image_in.cols*image_in.rows), 1, 0.7);
  mser->detectRegions(image_in, contours, bboxes);
  cv::Mat drawing = cv::Mat::zeros( image_in.size(), CV_8UC3 );
  drawing = drawing + image_in;

  for (int i = 0; i < bboxes.size(); i++)
  {
    cv::rectangle(drawing, bboxes[i], CV_RGB(0, 255, 0));
  }

  cv::namedWindow("MSER");
  cv::imshow("MSER", drawing);
  cv::waitKey(1);
}

void SignRecognition::detectORB(cv::Mat& image_in)
{
  for (auto keypointIterator = source_keypoints_.begin(); keypointIterator != source_keypoints_.end(); ++keypointIterator)
  {
    std::vector<cv::KeyPoint> current_keypoints = keypointIterator->second;
    cv::Mat current_descriptor = source_descriptors_[keypointIterator->first];
    matchImages(current_keypoints, current_descriptor,image_in, points_3d_, points_2d_, images_source_[keypointIterator->first]);

    if (points_3d_.size()>5 && points_2d_.size()>5)
    {
      my_pnpsolver_.estimatePoseRANSAC(points_3d_, points_2d_, pnpMethod_, inliers_idx_, iterationscount_, reprojectionError_, confidence_);

      // draw coordinate axis overlay
      my_graphicshandler_.drawCoordinateAxis(my_pnpsolver_.getRMat(),my_pnpsolver_.gettvec(),my_pnpsolver_.getCameraMatrix(),my_pnpsolver_.getDistCoeffs(),image_in);
      cv::imshow("Camera output",image_in);
    }
    else
    {
      cv::imshow("Camera output",image_in);
    }

    points_3d_.clear();
    points_2d_.clear();
    if( cv::waitKey(1)=='b') break;
  }
}

// Compute keypoints and descriptors for a vector of source images
void SignRecognition::describeImages(std::vector<cv::Mat>& images_source, std::map<int,std::vector<cv::KeyPoint> >& source_keypoints, std::map<int,cv::Mat>& source_descriptors)
{
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  for (auto it_image = images_source.begin(); it_image != images_source.end(); ++it_image)
  {
    my_detectormatcher_.computeKeyPoints(*it_image,keypoints);
    my_detectormatcher_.computeDescriptors(*it_image,keypoints,descriptors);

    if (!descriptors.empty() && !keypoints.empty())
    {
      // append keypoints and descriptors
      source_keypoints.insert(std::make_pair(it_image-images_source.begin(),keypoints));
      source_descriptors.insert(std::make_pair(it_image-images_source.begin(),descriptors));
    }
  }
}

// match a provided set of descriptors and keypoints with an image frame and save the matched points in a vector of 2d and 3d point matches
void SignRecognition::matchImages(std::vector<cv::KeyPoint>& source_keypoints, cv::Mat& source_descriptors, cv::Mat& frame_image, std::vector<cv::Point3f>& points_3d, std::vector<cv::Point2f>& points_2d, cv::Mat& source_image)
{
  std::vector<cv::KeyPoint> frame_keypoints;
  std::vector<cv::Mat> frame_descriptors;
  std::vector<cv::DMatch> good_matches;

  // detect keypoints in frame and match them to the provided source descriptors
  my_detectormatcher_.fastRobustMatch(frame_image,good_matches,frame_keypoints,source_descriptors);

  // DEBUG: draw matches
  cv::Mat test;
  cv::drawMatches(frame_image,frame_keypoints,source_image,source_keypoints,good_matches,test);
  cv::namedWindow("Matches",CV_WINDOW_NORMAL);
  cv::imshow("Matches",test);
  cv::waitKey(0);

  //my_detectormatcher_.robustMatch(frame_image,good_matches,frame_keypoints,source_descriptors);
  if (good_matches.size()>3)
  {
    for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
    {
      cv::Point3f point3d_model = (cv::Point3f) source_keypoints[ good_matches[match_index].trainIdx ].pt;
      cv::Point2f point2d_scene = frame_keypoints[ good_matches[match_index].queryIdx ].pt;
      points_3d.push_back(point3d_model);
      points_2d.push_back(point2d_scene);
    }
  }
}


SignRecognition::~SignRecognition() {
}
