/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef EXAMPLE_MARKERLESS_AR_PATTERN_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERN_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

#include <opencv2/opencv.hpp>

/**
 * Store the image data and computed descriptors of target pattern
 */
struct Pattern
{
  cv::Size                  size;
  cv::Mat                   frame;
  cv::Mat                   grayImg;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat                   descriptors;

  std::vector<cv::Point2f>  points2d;
  std::vector<cv::Point3f>  points3d;
};

/**
 * Intermediate pattern tracking info structure
 */
struct PatternTrackingInfo
{
  cv::Mat                   homography;
  std::vector<cv::Point2f>  points2d;
  Transformation            pose3d;

  void draw2dContour(cv::Mat& image, cv::Scalar color) const;

  /**
   * Compute pattern pose using PnP algorithm
   */
  void computePose(const Pattern& pattern, const CameraCalibration& calibration);
};

#endif