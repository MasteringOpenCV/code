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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "IDistance.h"

#include "Common.h"


/**
 Feature Matching Interface
 */
class IFeatureMatcher {
public:
	virtual void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches) = 0;
	virtual std::vector<cv::KeyPoint> GetImagePoints(int idx) = 0;
};
