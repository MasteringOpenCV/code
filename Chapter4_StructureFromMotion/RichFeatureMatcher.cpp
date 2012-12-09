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

#include "RichFeatureMatcher.h"

#include "FindCameraMatrices.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <set>

using namespace std;
using namespace cv;

//c'tor
RichFeatureMatcher::RichFeatureMatcher(std::vector<cv::Mat>& imgs_, 
									   std::vector<std::vector<cv::KeyPoint> >& imgpts_) :
	imgpts(imgpts_), imgs(imgs_)
{
	detector = FeatureDetector::create("PyramidFAST");
	extractor = DescriptorExtractor::create("ORB");
	
	std::cout << " -------------------- extract feature points for all images -------------------\n";
	detector->detect(imgs, imgpts);
	extractor->compute(imgs, imgpts, descriptors);
	std::cout << " ------------------------------------- done -----------------------------------\n";
}	

void RichFeatureMatcher::MatchFeatures(int idx_i, int idx_j, vector<DMatch>* matches) {
    
#ifdef __SFM__DEBUG__
    const Mat& img_1 = imgs[idx_i];
    const Mat& img_2 = imgs[idx_j];
#endif
    const vector<KeyPoint>& imgpts1 = imgpts[idx_i];
    const vector<KeyPoint>& imgpts2 = imgpts[idx_j];
    const Mat& descriptors_1 = descriptors[idx_i];
    const Mat& descriptors_2 = descriptors[idx_j];
    
    std::vector< DMatch > good_matches_,very_good_matches_;
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    
	stringstream ss; ss << "imgpts1 has " << imgpts1.size() << " points (descriptors " << descriptors_1.rows << ")" << endl;
    cout << ss.str();
	stringstream ss1; ss1 << "imgpts2 has " << imgpts2.size() << " points (descriptors " << descriptors_2.rows << ")" << endl;
    cout << ss1.str();
    
    keypoints_1 = imgpts1;
    keypoints_2 = imgpts2;
    
    if(descriptors_1.empty()) {
        CV_Error(0,"descriptors_1 is empty");
    }
    if(descriptors_2.empty()) {
        CV_Error(0,"descriptors_2 is empty");
    }
    
    //matching descriptor vectors using Brute Force matcher
    BFMatcher matcher(NORM_HAMMING,true); //allow cross-check. use Hamming distance for binary descriptor (ORB)
    std::vector< DMatch > matches_;
    if (matches == NULL) {
        matches = &matches_;
    }
    if (matches->size() == 0) {
        matcher.match( descriptors_1, descriptors_2, *matches );
    }
    
	assert(matches->size() > 0);
	
//    double max_dist = 0; double min_dist = 1000.0;
//    //-- Quick calculation of max and min distances between keypoints
//    for(unsigned int i = 0; i < matches->size(); i++ )
//    { 
//        double dist = (*matches)[i].distance;
//		if (dist>1000.0) { dist = 1000.0; }
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
//    
//#ifdef __SFM__DEBUG__
//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );
//#endif
    
    vector<KeyPoint> imgpts1_good,imgpts2_good;
    
//    if (min_dist <= 0) {
//        min_dist = 10.0;
//    }
    
    // Eliminate any re-matching of training points (multiple queries to one training)
//    double cutoff = 4.0*min_dist;
    std::set<int> existing_trainIdx;
    for(unsigned int i = 0; i < matches->size(); i++ )
    { 
        //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
        if ((*matches)[i].trainIdx <= 0) {
            (*matches)[i].trainIdx = (*matches)[i].imgIdx;
        }
        
        if( existing_trainIdx.find((*matches)[i].trainIdx) == existing_trainIdx.end() && 
           (*matches)[i].trainIdx >= 0 && (*matches)[i].trainIdx < (int)(keypoints_2.size()) /*&&
           (*matches)[i].distance > 0.0 && (*matches)[i].distance < cutoff*/ ) 
        {
            good_matches_.push_back( (*matches)[i]);
            imgpts1_good.push_back(keypoints_1[(*matches)[i].queryIdx]);
            imgpts2_good.push_back(keypoints_2[(*matches)[i].trainIdx]);
            existing_trainIdx.insert((*matches)[i].trainIdx);
        }
    }
    
    
#ifdef __SFM__DEBUG__
    cout << "keypoints_1.size() " << keypoints_1.size() << " imgpts1_good.size() " << imgpts1_good.size() << endl;
    cout << "keypoints_2.size() " << keypoints_2.size() << " imgpts2_good.size() " << imgpts2_good.size() << endl;
    
    {
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                    good_matches_, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );        
        //-- Show detected matches
		stringstream ss; ss << "Feature Matches " << idx_i << "-" << idx_j;
        imshow(ss.str() , img_matches );
        waitKey(500);
        destroyWindow(ss.str());
    }
#endif
    
    vector<uchar> status;
    vector<KeyPoint> imgpts2_very_good,imgpts1_very_good;
    
    assert(imgpts1_good.size() > 0);
	assert(imgpts2_good.size() > 0);
	assert(good_matches_.size() > 0);
	assert(imgpts1_good.size() == imgpts2_good.size() && imgpts1_good.size() == good_matches_.size());
	
    //Select features that make epipolar sense
    GetFundamentalMat(keypoints_1,keypoints_2,imgpts1_very_good,imgpts2_very_good,good_matches_);
	
    //Draw matches
#ifdef __SFM__DEBUG__
    {
        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                    good_matches_, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );        
        //-- Show detected matches
        imshow( "Good Matches", img_matches );
        waitKey(100);
        destroyWindow("Good Matches");
    }
#endif
}
