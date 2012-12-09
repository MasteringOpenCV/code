/*****************************************************************************
*   MarkerDetectionNativeImpl.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_MarkerDetectionNativeImpl_hpp
#define Example_MarkerBasedAR_MarkerDetectionNativeImpl_hpp

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <vector>
#include <memory>

////////////////////////////////////////////////////////////////////
// File includes:
#include "BGRAVideoFrame.h"
#include "GeometryTypes.hpp"

class MarkerDetectionFacade
{
public:
  virtual void processFrame(const BGRAVideoFrame& frame) = 0;
  virtual const std::vector<Transformation>& getTransformations() const = 0;
  virtual ~MarkerDetectionFacade() {}
};

std::auto_ptr<MarkerDetectionFacade> createMarkerDetection(CameraCalibration calibration);

#endif
