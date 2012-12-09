/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#pragma once

#define STRATEGY_USE_OPTICAL_FLOW		1
#define STRATEGY_USE_DENSE_OF			2
#define STRATEGY_USE_FEATURE_MATCH		4
#define STRATEGY_USE_HORIZ_DISPARITY	8

class IDistance {
public:
	virtual void OnlyMatchFeatures(int strategy = STRATEGY_USE_OPTICAL_FLOW + STRATEGY_USE_DENSE_OF + STRATEGY_USE_FEATURE_MATCH) = 0;
	virtual void RecoverDepthFromImages() = 0;
	virtual std::vector<cv::Point3d> getPointCloud() = 0;
	virtual const std::vector<cv::Vec3b>& getPointCloudRGB() = 0;
};
