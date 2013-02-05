/*****************************************************************************
*   CameraCalibration.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_CameraCalibration_hpp
#define Example_MarkerBasedAR_CameraCalibration_hpp

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

/**
 * A camera calibraiton class that stores intrinsic matrix
 * and distorsion vector.
 */
class CameraCalibration
{
public:
  CameraCalibration();
  CameraCalibration(float fx, float fy, float cx, float cy);
  CameraCalibration(float fx, float fy, float cx, float cy, float distorsionCoeff[4]);
  
  void getMatrix34(float cparam[3][4]) const;

  const Matrix33& getIntrinsic() const;
  const Vector4&  getDistorsion() const;
  
private:
  Matrix33 m_intrinsic;
  Vector4  m_distorsion;
};

#endif
