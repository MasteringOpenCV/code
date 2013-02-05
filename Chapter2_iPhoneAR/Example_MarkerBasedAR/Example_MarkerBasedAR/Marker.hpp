/*****************************************************************************
*   Marker.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_Marker_hpp
#define Example_MarkerBasedAR_Marker_hpp

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

/**
 * This class represents a marker
 */
class Marker
{  
public:
  Marker();
  
  friend bool operator<(const Marker &M1,const Marker&M2);
  friend std::ostream & operator<<(std::ostream &str,const Marker &M);

  static cv::Mat rotate(cv::Mat  in);
  static int hammDistMarker(cv::Mat bits);
  static int mat2id(const cv::Mat &bits);
  static int getMarkerId(cv::Mat &in,int &nRotations);
  
public:
  
  // Id of  the marker
  int id;
  
  // Marker transformation with regards to the camera
  Transformation transformation;
  
  std::vector<cv::Point2f> points;

  // Helper function to draw the marker contour over the image
  void drawContour(cv::Mat& image, cv::Scalar color = CV_RGB(0,250,0)) const;
};

#endif
