/*****************************************************************************
*   MarkerDetector.cpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <iostream>

////////////////////////////////////////////////////////////////////
// File includes:
#include "MarkerDetector.hpp"
#include "Marker.hpp"
#include "TinyLA.hpp"

std::auto_ptr<MarkerDetectionFacade> createMarkerDetection(CameraCalibration calibration)
{
  return std::auto_ptr<MarkerDetectionFacade>(new MarkerDetector(calibration));
}

MarkerDetector::MarkerDetector(CameraCalibration calibration)
: m_minContourLengthAllowed(100)
, markerSize(100,100)
{
  cv::Mat(3,3, CV_32F, const_cast<float*>(&calibration.getIntrinsic().data[0])).copyTo(camMatrix);
  cv::Mat(4,1, CV_32F, const_cast<float*>(&calibration.getDistorsion().data[0])).copyTo(distCoeff);
  
  bool centerOrigin = true;
  if (centerOrigin)
  {
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
    m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
  }
  else
  {
    m_markerCorners3d.push_back(cv::Point3f(0,0,0));
    m_markerCorners3d.push_back(cv::Point3f(1,0,0));
    m_markerCorners3d.push_back(cv::Point3f(1,1,0));
    m_markerCorners3d.push_back(cv::Point3f(0,1,0));    
  }
    
  m_markerCorners2d.push_back(cv::Point2f(0,0));
  m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,0));
  m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,markerSize.height-1));
  m_markerCorners2d.push_back(cv::Point2f(0,markerSize.height-1));
}

void MarkerDetector::processFrame(const BGRAVideoFrame& frame)
{
  std::vector<Marker> markers;
  findMarkers(frame, markers);
  
  m_transformations.clear();
  for (size_t i=0; i<markers.size(); i++)
  {
    m_transformations.push_back(markers[i].transformation);
  }
}

const std::vector<Transformation>& MarkerDetector::getTransformations() const
{
  return m_transformations;
}


bool MarkerDetector::findMarkers(const BGRAVideoFrame& frame, std::vector<Marker>& detectedMarkers)
{
  cv::Mat bgraMat(frame.height, frame.width, CV_8UC4, frame.data, frame.stride);
  
  // Convert the image to grayscale
  prepareImage(bgraMat, m_grayscaleImage);
  
  // Make it binary
  performThreshold(m_grayscaleImage, m_thresholdImg);
  
  // Detect contours
  findContours(m_thresholdImg, m_contours, m_grayscaleImage.cols / 5);
  
  // Find closed contours that can be approximated with 4 points
  findMarkerCandidates(m_contours, detectedMarkers);
  
  // Find is them are markers
  detectMarkers(m_grayscaleImage, detectedMarkers);

  // Calcualte their poses
  estimatePosition(detectedMarkers);

  //sort by id
  std::sort(detectedMarkers.begin(), detectedMarkers.end());
  return false;
}

void MarkerDetector::prepareImage(const cv::Mat& bgraMat, cv::Mat& grayscale)
{
  // Convert to grayscale
  cv::cvtColor(bgraMat, grayscale, CV_BGRA2GRAY);
}

void MarkerDetector::performThreshold(const cv::Mat& grayscale, cv::Mat& thresholdImg)
{
  cv::adaptiveThreshold(grayscale,   // Input image
                        thresholdImg,// Result binary image
                        255,         // 
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C, //
                        cv::THRESH_BINARY_INV, //
                        7, //
                        7  //
                        );
}

void MarkerDetector::findContours(const cv::Mat& thresholdImg, std::vector<std::vector<cv::Point> >& contours, int minContourPointsAllowed)
{
  cv::Mat thresholdImgCopy;
  thresholdImg.copyTo(thresholdImgCopy);
  
  std::vector<std::vector<cv::Point> > allContours;
  cv::findContours(thresholdImgCopy, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  
  contours.clear();
  for (size_t i=0;i<allContours.size();i++)
  {
    int contourSize = allContours[i].size();
    if (contourSize > minContourPointsAllowed)
    {
      contours.push_back(allContours[i]);
    }
  }
}

void MarkerDetector::findMarkerCandidates(const std::vector<std::vector<cv::Point> >& contours, 
                                          std::vector<Marker>& detectedMarkers)
{
  std::vector<cv::Point>  approxCurve;
  
  std::vector<Marker> possibleMarkers;
  
  // For each contour, analyze if it is a paralelepiped likely to be the marker
  for (size_t i=0; i<contours.size(); i++)
  {
    // Approximate to a poligon
    cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size())*0.05 , true);
    
    // We interested only in polygons that contains only four vertices
    if (approxCurve.size() != 4)
      continue;
    
    // And they have to be convex
    if (!cv::isContourConvex(approxCurve))
      continue;
    
    // Ensure that the distace between consecutive points is large enough
    float minDist = 1e10;
    for (int i=0;i<4;i++)
    {
      cv::Point vec = approxCurve[i] - approxCurve[(i+1)%4];            
      float squaredDistance = vec.dot(vec);
      minDist = std::min(minDist,squaredDistance);
    }
    
    // Check that distance is not very small
    if (minDist > m_minContourLengthAllowed)
    {
      Marker m;
      for(int i=0;i<4;i++)
      {
        m.points.push_back( cv::Point2f(approxCurve[i].x,approxCurve[i].y) );
      }
      possibleMarkers.push_back(m);
    }
  } 
  
  //sort the points in anti-clockwise order
  for (size_t i=0; i<possibleMarkers.size(); i++)
  {
    Marker& marker = possibleMarkers[i];
    
    //trace a line between the first and second point.
    //if the thrid point is at the right side, then the points are anti-clockwise
    cv::Point v1 = marker.points[1] - marker.points[0];
    cv::Point v2 = marker.points[2] - marker.points[0];
                                                
    double o = (v1.x * v2.y) - (v1.y * v2.x);
    
    if (o  < 0.0)         //if the third point is in the left side, then sort in anti-clockwise order
    {
      std::swap(marker.points[1],marker.points[3]);
    }
  } 
  
  // remove these elements whise corners are too close to each other first detect candidates
  std::vector< std::pair<int,int> > tooNearCandidates;
  for (size_t i=0;i<possibleMarkers.size();i++)
  { 
    const Marker& m1 = possibleMarkers[i];

    //calculate the average distance of each corner to the nearest corner of the other marker candidate
    for (size_t j=i+1;j<possibleMarkers.size();j++)
    {
      const Marker& m2 = possibleMarkers[j];
      
      float distSquared = 0;
      
      for(int c=0;c<4;c++)
      {
        cv::Point v = m1.points[c] - m2.points[c];
        distSquared += v.dot(v);
      }
      
      distSquared /= 4;

      if (distSquared < 100)
      {
        tooNearCandidates.push_back(std::pair<int,int>(i,j));
      }
    }                
  }
  
  //mark for removal the element of  the pair with smaller perimeter
  std::vector<bool> removalMask (possibleMarkers.size(), false);
  
  for (size_t i=0;i<tooNearCandidates.size();i++)
  {
    float p1 = perimeter(possibleMarkers[tooNearCandidates[i].first ].points);
    float p2 = perimeter(possibleMarkers[tooNearCandidates[i].second].points);
    
    size_t removalIndex;
    if (p1 > p2)
      removalIndex = tooNearCandidates[i].second;
    else
      removalIndex = tooNearCandidates[i].first;
    
    removalMask[removalIndex] = true;
  }
  
  // Return candidates
  detectedMarkers.clear();
  for (size_t i=0;i<possibleMarkers.size();i++)
  {
    if (!removalMask[i])
      detectedMarkers.push_back(possibleMarkers[i]);
  }
}

void MarkerDetector::detectMarkers(const cv::Mat& grayscale, std::vector<Marker>& detectedMarkers)
{
  cv::Mat canonicalMarker;

  std::vector<Marker> goodMarkers;
  
  // Identify the markers
  for (size_t i=0;i<detectedMarkers.size();i++)
  {
    Marker& marker = detectedMarkers[i];
    
    // Find the perspective transfomation that brings current marker to rectangular form
    cv::Mat M = cv::getPerspectiveTransform(marker.points, m_markerCorners2d);
  
    // Transform image to get a canonical marker image
    cv::warpPerspective(grayscale, canonicalMarker,  M, markerSize);
        
    int nRotations;
    int id = Marker::getMarkerId(canonicalMarker,nRotations);
    if (id !=- 1)
    {
      marker.id = id;
      //sort the points so that they are always in the same order no matter the camera orientation
      std::rotate(marker.points.begin(), marker.points.begin() + 4 - nRotations, marker.points.end());
      
      goodMarkers.push_back(marker);
    }
  }  
  
  //refine using subpixel accuracy the corners
  if (goodMarkers.size() > 0)
  {
    std::vector<cv::Point2f> preciseCorners(4 * goodMarkers.size());
    
    for (size_t i=0; i<goodMarkers.size(); i++)
    {  
      Marker& marker = goodMarkers[i];      
      
      for (int c=0;c<4;c++)
      {
        preciseCorners[i*4+c] = marker.points[c];
      }
    }
    
    cv::cornerSubPix(grayscale, preciseCorners, cvSize(5,5), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER,30,0.1));
    
    //copy back
    for (size_t i=0;i<goodMarkers.size();i++)
    {
      Marker& marker = goodMarkers[i];      
      
      for (int c=0;c<4;c++) 
      {
        marker.points[c] = preciseCorners[i*4+c];
      }      
    }
  }
  
  detectedMarkers = goodMarkers;
}


void MarkerDetector::estimatePosition(std::vector<Marker>& detectedMarkers)
{
  for (size_t i=0; i<detectedMarkers.size(); i++)
  {                    
    Marker& m = detectedMarkers[i];
    
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux,taux;
    cv::solvePnP(m_markerCorners3d, m.points, camMatrix, distCoeff,raux,taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);
    
    cv::Mat_<float> rotMat(3,3); 
    cv::Rodrigues(Rvec, rotMat);
    
    // Copy to transformation matrix
    m.transformation = Transformation();
    
    for (int col=0; col<3; col++)
    {
      for (int row=0; row<3; row++)
      {        
        m.transformation.r().mat[row][col] = rotMat(row,col); // Copy rotation component
      }
      m.transformation.t().data[col] = Tvec(col); // Copy translation component
    }
    
    // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
    m.transformation = m.transformation.getInverted();
  }
}


