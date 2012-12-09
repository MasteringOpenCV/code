/*****************************************************************************
*   TinyLA.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_TinyLA_hpp
#define Example_MarkerBasedAR_TinyLA_hpp

#include <vector>
#include <opencv2/opencv.hpp>

float perimeter(const std::vector<cv::Point2f> &a);

bool isInto(cv::Mat &contour, std::vector<cv::Point2f> &b);

#endif
