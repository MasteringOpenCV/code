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

#include "OFFeatureMatcher.h"
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>

#ifdef __SFM__DEBUG__
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <sstream>
#endif

#include <set>

using namespace std;
using namespace cv;

//c'tor
OFFeatureMatcher::OFFeatureMatcher(
	bool _use_gpu,
	std::vector<cv::Mat>& imgs_, 
	std::vector<std::vector<cv::KeyPoint> >& imgpts_) :
AbstractFeatureMatcher(_use_gpu),imgpts(imgpts_), imgs(imgs_)
{
	//detect keypoints for all images
	FastFeatureDetector ffd;
//	DenseFeatureDetector ffd;
	ffd.detect(imgs, imgpts);
}

void OFFeatureMatcher::MatchFeatures(int idx_i, int idx_j, vector<DMatch>* matches) {	
	vector<Point2f> i_pts; 
	KeyPointsToPoints(imgpts[idx_i],i_pts);
	
	vector<Point2f> j_pts(i_pts.size());
	
	// making sure images are grayscale
	Mat prevgray,gray;
	if (imgs[idx_i].channels() == 3) {
		cvtColor(imgs[idx_i],prevgray,CV_RGB2GRAY);
		cvtColor(imgs[idx_j],gray,CV_RGB2GRAY);
	} else {
		prevgray = imgs[idx_i];
		gray = imgs[idx_j];
	}

	vector<uchar> vstatus(i_pts.size()); vector<float> verror(i_pts.size());

	if(use_gpu) {
		gpu::GpuMat gpu_prevImg,gpu_nextImg,gpu_prevPts,gpu_nextPts,gpu_status,gpu_error;
		gpu_prevImg.upload(prevgray);
		gpu_nextImg.upload(gray);
		gpu_prevPts.upload(Mat(i_pts).t());

		gpu::PyrLKOpticalFlow gpu_of;
		gpu_of.sparse(gpu_prevImg,gpu_nextImg,gpu_prevPts,gpu_nextPts,gpu_status,&gpu_error);

		Mat j_pts_mat;
		gpu_nextPts.download(j_pts_mat);
		Mat(j_pts_mat.t()).copyTo(Mat(j_pts));

		Mat vstatus_mat,verror_mat;
		gpu_status.download(vstatus_mat);
		gpu_error.download(verror_mat);
		Mat(vstatus_mat.t()).copyTo(Mat(vstatus));
		Mat(verror_mat.t()).copyTo(Mat(verror));
	} else {
		CV_PROFILE("OpticalFlow",calcOpticalFlowPyrLK(prevgray, gray, i_pts, j_pts, vstatus, verror);)
	}

	double thresh = 1.0;
	vector<Point2f> to_find;
	vector<int> to_find_back_idx;
	for (unsigned int i=0; i<vstatus.size(); i++) {
		if (vstatus[i] && verror[i] < 12.0) {
			to_find_back_idx.push_back(i);
			to_find.push_back(j_pts[i]);
		} else {
			vstatus[i] = 0;
		}
	}

	std::set<int> found_in_imgpts_j;
	Mat to_find_flat = Mat(to_find).reshape(1,to_find.size());
	
	vector<Point2f> j_pts_to_find;
	KeyPointsToPoints(imgpts[idx_j],j_pts_to_find);
	Mat j_pts_flat = Mat(j_pts_to_find).reshape(1,j_pts_to_find.size());

	vector<vector<DMatch> > knn_matches;
	//FlannBasedMatcher matcher;
	BFMatcher matcher(CV_L2);
	CV_PROFILE("RadiusMatch",matcher.radiusMatch(to_find_flat,j_pts_flat,knn_matches,2.0f);)
	CV_PROFILE("Prune",
	for(int i=0;i<knn_matches.size();i++) {
		DMatch _m;
		if(knn_matches[i].size()==1) {
			_m = knn_matches[i][0];
		} else if(knn_matches[i].size()>1) {
			if(knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7) {
				_m = knn_matches[i][0];
			} else {
				continue; // did not pass ratio test
			}
		} else {
			continue; // no match
		}
		if (found_in_imgpts_j.find(_m.trainIdx) == found_in_imgpts_j.end()) { // prevent duplicates
			_m.queryIdx = to_find_back_idx[_m.queryIdx]; //back to original indexing of points for <i_idx>
			matches->push_back(_m);
			found_in_imgpts_j.insert(_m.trainIdx);
		}
	}
	)
	cout << "pruned " << matches->size() << " / " << knn_matches.size() << " matches" << endl;
#if 0
#ifdef __SFM__DEBUG__
	{
		// draw flow field
		Mat img_matches; cvtColor(imgs[idx_i],img_matches,CV_GRAY2BGR);
		i_pts.clear(); j_pts.clear();
		for(int i=0;i<matches->size();i++) {
			//if (i%2 != 0) {
//				continue;
//			}
			Point i_pt = imgpts[idx_i][(*matches)[i].queryIdx].pt;
			Point j_pt = imgpts[idx_j][(*matches)[i].trainIdx].pt;
			i_pts.push_back(i_pt);
			j_pts.push_back(j_pt);
			vstatus[i] = 1;
		}
		drawArrows(img_matches, i_pts, j_pts, vstatus, verror, Scalar(0,255));
		stringstream ss; 
		ss << matches->size() << " matches";
//		putText(img_matches,ss.str(),Point(10,20),CV_FONT_HERSHEY_PLAIN,1.0,Scalar(255),2);
		ss.clear(); ss << "flow_field_" << omp_get_thread_num() << ".png";
		imshow( ss.str(), img_matches );
		int c = waitKey(0);
		if (c=='s') {
			imwrite(ss.str(), img_matches);
		}
		destroyWindow(ss.str());
	}
#endif
#endif
}
