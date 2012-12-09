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


#ifndef Example_MarkerBasedAR_CameraCalibration_hpp
#define Example_MarkerBasedAR_CameraCalibration_hpp

////////////////////////////////////////////////////////////////////
// File includes:
#include <opencv2/opencv.hpp>

/**
* A camera calibration class that stores intrinsic matrix and distortion coefficients.
*/
class CameraCalibration
{
public:
    CameraCalibration();
    CameraCalibration(float fx, float fy, float cx, float cy);
    CameraCalibration(float fx, float fy, float cx, float cy, float distorsionCoeff[5]);

    void getMatrix34(float cparam[3][4]) const;

    const cv::Matx33f& getIntrinsic() const;
    const cv::Mat_<float>&  getDistorsion() const;

    float& fx();
    float& fy();

    float& cx();
    float& cy();

    float fx() const;
    float fy() const;

    float cx() const;
    float cy() const;
private:
    cv::Matx33f     m_intrinsic;
    cv::Mat_<float> m_distortion;
};

#endif
