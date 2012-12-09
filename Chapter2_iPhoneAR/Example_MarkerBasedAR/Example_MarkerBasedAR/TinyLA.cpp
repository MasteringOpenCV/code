/*****************************************************************************
*   TinyLA.cpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include "TinyLA.hpp"

float perimeter(const std::vector<cv::Point2f> &a)
{
  float sum=0, dx, dy;
  
  for (size_t i=0;i<a.size();i++)
  {
    size_t i2=(i+1) % a.size();
    
    dx = a[i].x - a[i2].x;
    dy = a[i].y - a[i2].y;
    
    sum += sqrt(dx*dx + dy*dy);
  }
  
  return sum;
}


bool isInto(cv::Mat &contour, std::vector<cv::Point2f> &b)
{
  for (size_t i=0;i<b.size();i++)
  {
    if (cv::pointPolygonTest( contour,b[i],false)>0) return true;
  }
  return false;
}
