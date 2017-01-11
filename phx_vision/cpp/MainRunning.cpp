/*
 * MainRunning.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: mykyta_denysov
 */

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <utility>
#include <getopt.h>

#include "MainRunning.h"
#include <opencv2/opencv.hpp>

#include "DetectorMatcher.h"
#include "PnPSolver.h"
#include "GraphicsHandler.h"

Main_Running::Main_Running() {

//	if (argc<1)
//	{
//		printf("Not enough inputs provided for operation");
//		exit(EXIT_FAILURE);
//	}

    PnPSolver my_pnpsolver_ = PnPSolver();

	std::string const ending = ".csv";
	std::string current;
	std::string current_argument;

	cv::Mat image_current;
	std::vector<cv::Mat> images_source_;

    images_source_.push_back(cv::imread("/home/mykyta_denysov/Pictures/heart.png"));
    images_source_.push_back(cv::imread("/home/mykyta_denysov/Pictures/orange_PNG805.png"));


    if(!images_source_.front().data)
    {
        std::cout << "Could not open or find the image" << std::endl;
        exit(EXIT_FAILURE);
    }

//    cv::imshow("Display window", images_source_.front() );
//    cv::waitKey(0);

//    static struct option optionnames[] = { {"camera", required_argument, 0, 'o'}, {"images", required_argument, 0, 'i'} };
//    int option_index = 0;

//	// handle console arguments
//    while (console_option = getopt_long(argc, argv, "", optionnames, option_index)!=-1)
//    {
//        switch (console_option)
//        {
//            case
//        }
//    }


//	for (int n_input = 0; n_input<argc; ++n_input)
//	{
//		// read out camera calibration parameter values from .csv file provided after --matrix console parameter (mandatory)
//		current = std::string(argv[n_input]);
//		if(current == "--matrix")
//		{
//			if(n_input+1>=argc)
//			{
//				printf("Camera matrix out of bounds!");
//				exit(EXIT_FAILURE);
//			}
//			else
//			{
//				current_argument = std::string(argv[n_input+1]);

//				if (0 == current_argument.compare(0,1,"--"))
//				{
//						printf("WARNING: Matrix argument is empty!");
//						exit(EXIT_FAILURE);
//				}

//				// check if the provided parameter string ends with ".csv"
//				if (current_argument.length() > ending.length())
//				{
//					if (0==current_argument.compare(current_argument.length() - ending.length(), ending.length(), ending))
//					{
//						this -> my_pnpsolver_.loadCSVdata(current_argument);
//						n_input = n_input + 2;
//					}
//				}
//			}
//		}

//		// read out images prefix string priovided after --images console parameter (mandatory)
//		else if(current == "--images")
//		{
//			if(n_input+1>=argc)
//			{
//				printf("Image prefix out of bounds!");
//				exit(EXIT_FAILURE);
//			}

//			for (int n_image  = n_input+1;n_image<argc; ++n_image)
//			{
//				current_argument = std::string(argv[n_image+1]);
//				// break if another argument follows
//				if (0 == current_argument.compare(0,1,"--"))
//				{
//					break;
//				}

//				image_current = cv::imread(current_argument,0);
//				if (image_current.empty()){
//					printf(strcat("Unable to read image ", current_argument.c_str()));
//					continue;
//				}
//				images_source_.push_back(image_current);
//				// todo: set jump only at the end of loop
//				n_input = n_image;
//			}

//		}

//	}

	// get source image keypoints and descriptors
    this -> describeImages(this->images_source_,this->source_keypoints_,this->source_descriptors_);

    std::cout<<"Number of keypoints detected: "<<this->source_keypoints_.size()<<"Number of descriptors: "<<this->source_descriptors_.size();
    exit(EXIT_FAILURE);

	// grab webcam
	cv::VideoCapture webcam;

	if(!webcam.open(0))
	{
		printf("Unable to open webcam 0!");
		exit(EXIT_FAILURE);
	}

	cv::Mat frame_webcam;

	// map container for matches
	std::map<int,cv::DMatch> good_matches;

	// matched point containers
	std::vector<cv::Point3f> points_3d;
	std::vector<cv::Point2f> points_2d;

	// pnp solve parameters
	int pnpMethod = cv::SOLVEPNP_ITERATIVE;
	cv::Mat inliers_idx;
	int iterationscount = 500;
	float reprojectionError = 2.0;
	double confidence = 0.95;

	for (;;)
	{
		webcam >> frame_webcam;
		if (frame_webcam.empty()) {break;}

		//todo: create loop for images
        for (std::map<int,std::vector<cv::KeyPoint> >::const_iterator keypointIterator = source_keypoints_.begin(); keypointIterator != source_keypoints_.end(); ++keypointIterator)
		{
            printf("Succesfully arrived at webcam loop");

			std::vector<cv::KeyPoint> current_keypoints = keypointIterator->second;
			cv::Mat current_descriptor = source_descriptors_[keypointIterator->first];

            this->matchImages(current_keypoints, current_descriptor,frame_webcam, points_3d, points_2d);

			// solve point correspondence
			my_pnpsolver_.estimatePoseRANSAC(points_3d, points_2d, pnpMethod, inliers_idx, iterationscount, reprojectionError, confidence);

			// draw coordinate axis overlay
			//todo: create GUI class that draws axis for projection
			cv::Mat frame_graphics = my_graphicshandler_.drawCoordinateAxis(my_pnpsolver_.getRMat(),my_pnpsolver_.gettvec(),my_pnpsolver_.getCameraMatrix(),my_pnpsolver_.getDistCoeffs(),frame_webcam);
			cv::imshow("Camera output",frame_graphics);

			points_3d.clear();
			points_2d.clear();
			if( cv::waitKey(1)=='b') break;
		}
	}
}

// Compute keypoints and descriptors for a vector of source images
void Main_Running::describeImages(std::vector<cv::Mat>& images_source, std::map<int,std::vector<cv::KeyPoint> >& source_keypoints, std::map<int,cv::Mat>& source_descriptors)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;

    int index = 0;
    std::cout<<"Arrived at descriptor part"<<std::endl;
    //for (std::vector<cv::Mat>::const_iterator ImageIterator = images_source_.begin(); ImageIterator != images_source_.end() + 1; ++ImageIterator)
    for (unsigned int i=0; i < images_source.size(); i++)
    {
        std::cout<<"In loop!"<<std::endl;

        cv::imshow("Display window", images_source[i] );
        cv::waitKey(0);

        my_detectormatcher_.computeKeyPoints(*ImageIterator,keypoints);
        std::cout<<"Found following number of keypoints: "<< keypoints.size() <<std::endl;
		my_detectormatcher_.computeDescriptors(*ImageIterator,keypoints,descriptor);
        std::cout<<"Found following number of descriptors: "<< descriptor.size() <<std::endl;


		if (!descriptor.empty())
		{
			if (!keypoints.empty())
			{
				// append good matches to the vectors
				source_keypoints.insert(std::pair<int,std::vector<cv::KeyPoint> >(index,keypoints));
				source_descriptors.insert(std::pair<int,cv::Mat>(index,descriptor));
			}
		}

        // clear placeholder objects
        descriptor.release();
        keypoints.clear();
		index++;
	}
}

// match a provided set of descriptors and keypoints with an image frame and save the matched points in a vector of 2d and 3d point matches
void Main_Running::matchImages(std::vector<cv::KeyPoint>& source_keypoints, cv::Mat& source_descriptors, cv::Mat& frame_image, std::vector<cv::Point3f>& points_3d, std::vector<cv::Point2f>& points_2d)
{
	std::vector<cv::KeyPoint> frame_keypoints;
	std::vector<cv::Mat> frame_descriptors;
    std::vector<cv::DMatch> good_matches;

    // detect keypoints in frame and match them to the provided source descriptors
    my_detectormatcher_.fastRobustMatch(frame_image,good_matches,frame_keypoints,source_descriptors);
    // solve PnP and draw coordinate overlay if enough matches found
    if (good_matches.size()>10)
    {
    	for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
    	{
    		//todo: checken ob das so richtig ist
    		cv::Point3f point3d_model = (cv::Point3f) source_keypoints[ good_matches[match_index].trainIdx ].pt;  // 3D point from model
    		cv::Point2f point2d_scene = frame_keypoints[ good_matches[match_index].queryIdx ].pt; // 2D point from the scene
    		points_3d.push_back(point3d_model);         // add 3D point
    		points_2d.push_back(point2d_scene);         // add 2D point
    	}
    }
}


Main_Running::~Main_Running() {
	// TODO Auto-generated destructor stub
}

