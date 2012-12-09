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

#include <vector>
#include <opencv2/core/core.hpp>
#include "Common.h"

class BundleAdjuster {
public:
	void adjustBundle(std::vector<CloudPoint>& pointcloud, 
					  cv::Mat& cam_matrix,
					  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
					  std::map<int ,cv::Matx34d>& Pmats);
private:
	int Count2DMeasurements(const std::vector<CloudPoint>& pointcloud);
};
