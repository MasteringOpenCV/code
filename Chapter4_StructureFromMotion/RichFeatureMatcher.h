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

#include "IFeatureMatcher.h"

class RichFeatureMatcher : public IFeatureMatcher {
private:
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	
	std::vector<cv::Mat> descriptors;
	
	std::vector<cv::Mat>& imgs; 
	std::vector<std::vector<cv::KeyPoint> >& imgpts;
public:
	//c'tor
	RichFeatureMatcher(std::vector<cv::Mat>& imgs, 
					   std::vector<std::vector<cv::KeyPoint> >& imgpts);
	
	void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches = NULL);
	
	std::vector<cv::KeyPoint> GetImagePoints(int idx) { return imgpts[idx]; }
};
