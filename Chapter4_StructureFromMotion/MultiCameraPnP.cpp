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

#include "MultiCameraPnP.h"
#include "BundleAdjuster.h"

using namespace std;

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/calib3d/calib3d.hpp>

bool sort_by_first(pair<int,pair<int,int> > a, pair<int,pair<int,int> > b) { return a.first < b.first; }

//Following Snavely07 4.2 - find how many inliers are in the Homography between 2 views
int MultiCameraPnP::FindHomographyInliers2Views(int vi, int vj) 
{
	vector<cv::KeyPoint> ikpts,jkpts; vector<cv::Point2f> ipts,jpts;
	GetAlignedPointsFromMatch(imgpts[vi],imgpts[vj],matches_matrix[make_pair(vi,vj)],ikpts,jkpts);
	KeyPointsToPoints(ikpts,ipts); KeyPointsToPoints(jkpts,jpts);

	double minVal,maxVal; cv::minMaxIdx(ipts,&minVal,&maxVal); //TODO flatten point2d?? or it takes max of width and height

	vector<uchar> status;
	cv::Mat H = cv::findHomography(ipts,jpts,status,CV_RANSAC, 0.004 * maxVal); //threshold from Snavely07
	return cv::countNonZero(status); //number of inliers
}

/**
 * Get an initial 3D point cloud from 2 views only
 */
void MultiCameraPnP::GetBaseLineTriangulation() {
	std::cout << "=========================== Baseline triangulation ===========================\n";

	cv::Matx34d P(1,0,0,0,
				  0,1,0,0,
				  0,0,1,0),
				P1(1,0,0,0,
				   0,1,0,0,
				   0,0,1,0);
	
	std::vector<CloudPoint> tmp_pcloud;

	//sort pairwise matches to find the lowest Homography inliers [Snavely07 4.2]
	cout << "Find highest match...";
	list<pair<int,pair<int,int> > > matches_sizes;
	//TODO: parallelize!
	for(std::map<std::pair<int,int> ,std::vector<cv::DMatch> >::iterator i = matches_matrix.begin(); i != matches_matrix.end(); ++i) {
		if((*i).second.size() < 100)
			matches_sizes.push_back(make_pair(100,(*i).first));
		else {
			int Hinliers = FindHomographyInliers2Views((*i).first.first,(*i).first.second);
			int percent = (int)(((double)Hinliers) / ((double)(*i).second.size()) * 100.0);
			cout << "[" << (*i).first.first << "," << (*i).first.second << " = "<<percent<<"] ";
			matches_sizes.push_back(make_pair((int)percent,(*i).first));
		}
	}
	cout << endl;
	matches_sizes.sort(sort_by_first);

	//Reconstruct from two views
	bool goodF = false;
	int highest_pair = 0;
	m_first_view = m_second_view = 0;
	//reverse iterate by number of matches
	for(list<pair<int,pair<int,int> > >::iterator highest_pair = matches_sizes.begin(); 
		highest_pair != matches_sizes.end() && !goodF; 
		++highest_pair) 
	{
		m_second_view = (*highest_pair).second.second;
		m_first_view  = (*highest_pair).second.first;

		std::cout << " -------- " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << " -------- " <<std::endl;
		//what if reconstrcution of first two views is bad? fallback to another pair
		//See if the Fundamental Matrix between these two views is good
		goodF = FindCameraMatrices(K, Kinv, distortion_coeff,
			imgpts[m_first_view], 
			imgpts[m_second_view], 
			imgpts_good[m_first_view],
			imgpts_good[m_second_view], 
			P, 
			P1,
			matches_matrix[std::make_pair(m_first_view,m_second_view)],
			tmp_pcloud
#ifdef __SFM__DEBUG__
			,imgs[m_first_view],imgs[m_second_view]
#endif
		);
		if (goodF) {
			vector<CloudPoint> new_triangulated;
			vector<int> add_to_cloud;

			Pmats[m_first_view] = P;
			Pmats[m_second_view] = P1;

			bool good_triangulation = TriangulatePointsBetweenViews(m_second_view,m_first_view,new_triangulated,add_to_cloud);
			if(!good_triangulation || cv::countNonZero(add_to_cloud) < 10) {
				std::cout << "triangulation failed" << std::endl;
				goodF = false;
				Pmats[m_first_view] = 0;
				Pmats[m_second_view] = 0;
				m_second_view++;
			} else {
				std::cout << "before triangulation: " << pcloud.size();
				for (unsigned int j=0; j<add_to_cloud.size(); j++) {
					if(add_to_cloud[j] == 1)
						pcloud.push_back(new_triangulated[j]);
				}
				std::cout << " after " << pcloud.size() << std::endl;
			}				
		}
	}
		
	if (!goodF) {
		cerr << "Cannot find a good pair of images to obtain a baseline triangulation" << endl;
		exit(0);
	}
	
	cout << "Taking baseline from " << imgs_names[m_first_view] << " and " << imgs_names[m_second_view] << endl;
	
//	double reproj_error;
//	{
//		std::vector<cv::KeyPoint> pt_set1,pt_set2;
//		
//		std::vector<cv::DMatch> matches = matches_matrix[std::make_pair(m_first_view,m_second_view)];
//
//		GetAlignedPointsFromMatch(imgpts[m_first_view],imgpts[m_second_view],matches,pt_set1,pt_set2);
//		
//		pcloud.clear();
//		reproj_error = TriangulatePoints(pt_set1, 
//										 pt_set2, 
//										 Kinv, 
//										 distortion_coeff,
//										 Pmats[m_first_view], 
//										 Pmats[m_second_view], 
//										 pcloud, 
//										 correspImg1Pt);
//		
//		for (unsigned int i=0; i<pcloud.size(); i++) {
//			pcloud[i].imgpt_for_img = std::vector<int>(imgs.size(),-1);
//			//matches[i] corresponds to pointcloud[i]
//			pcloud[i].imgpt_for_img[m_first_view] = matches[i].queryIdx;
//			pcloud[i].imgpt_for_img[m_second_view] = matches[i].trainIdx;
//		}
//	}
//	std::cout << "triangulation reproj error " << reproj_error << std::endl;
}

void MultiCameraPnP::Find2D3DCorrespondences(int working_view, 
	std::vector<cv::Point3f>& ppcloud, 
	std::vector<cv::Point2f>& imgPoints) 
{
	ppcloud.clear(); imgPoints.clear();

	vector<int> pcloud_status(pcloud.size(),0);
	for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view) 
	{
		int old_view = *done_view;
		//check for matches_from_old_to_working between i'th frame and <old_view>'th frame (and thus the current cloud)
		std::vector<cv::DMatch> matches_from_old_to_working = matches_matrix[std::make_pair(old_view,working_view)];

		for (unsigned int match_from_old_view=0; match_from_old_view < matches_from_old_to_working.size(); match_from_old_view++) {
			// the index of the matching point in <old_view>
			int idx_in_old_view = matches_from_old_to_working[match_from_old_view].queryIdx;

			//scan the existing cloud (pcloud) to see if this point from <old_view> exists
			for (unsigned int pcldp=0; pcldp<pcloud.size(); pcldp++) {
				// see if corresponding point was found in this point
				if (idx_in_old_view == pcloud[pcldp].imgpt_for_img[old_view] && pcloud_status[pcldp] == 0) //prevent duplicates
				{
					//3d point in cloud
					ppcloud.push_back(pcloud[pcldp].pt);
					//2d point in image i
					imgPoints.push_back(imgpts[working_view][matches_from_old_to_working[match_from_old_view].trainIdx].pt);

					pcloud_status[pcldp] = 1;
					break;
				}
			}
		}
	}
	cout << "found " << ppcloud.size() << " 3d-2d point correspondences"<<endl;
}

bool MultiCameraPnP::FindPoseEstimation(
	int working_view,
	cv::Mat_<double>& rvec,
	cv::Mat_<double>& t,
	cv::Mat_<double>& R,
	std::vector<cv::Point3f> ppcloud,
	std::vector<cv::Point2f> imgPoints
	) 
{
	if(ppcloud.size() <= 7 || imgPoints.size() <= 7 || ppcloud.size() != imgPoints.size()) { 
		//something went wrong aligning 3D to 2D points..
		cerr << "couldn't find [enough] corresponding cloud points... (only " << ppcloud.size() << ")" <<endl;
		return false;
	}

	vector<int> inliers;
	if(!use_gpu) {
		//use CPU
		double minVal,maxVal; cv::minMaxIdx(imgPoints,&minVal,&maxVal);
		CV_PROFILE("solvePnPRansac",cv::solvePnPRansac(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, 1000, 0.006 * maxVal, 0.25 * (double)(imgPoints.size()), inliers, CV_EPNP);)
		//CV_PROFILE("solvePnP",cv::solvePnP(ppcloud, imgPoints, K, distortion_coeff, rvec, t, true, CV_EPNP);)
	} else {
		//use GPU ransac
		//make sure datatstructures are cv::gpu compatible
		cv::Mat ppcloud_m(ppcloud); ppcloud_m = ppcloud_m.t();
		cv::Mat imgPoints_m(imgPoints); imgPoints_m = imgPoints_m.t();
		cv::Mat rvec_,t_;

		cv::gpu::solvePnPRansac(ppcloud_m,imgPoints_m,K_32f,distcoeff_32f,rvec_,t_,false);

		rvec_.convertTo(rvec,CV_64FC1);
		t_.convertTo(t,CV_64FC1);
	}

	vector<cv::Point2f> projected3D;
	cv::projectPoints(ppcloud, rvec, t, K, distortion_coeff, projected3D);

	if(inliers.size()==0) { //get inliers
		for(int i=0;i<projected3D.size();i++) {
			if(norm(projected3D[i]-imgPoints[i]) < 10.0)
				inliers.push_back(i);
		}
	}

#if 0
	//display reprojected points and matches
	cv::Mat reprojected; imgs_orig[working_view].copyTo(reprojected);
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::line(reprojected,imgPoints[ppt],projected3D[ppt],cv::Scalar(0,0,255),1);
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::line(reprojected,imgPoints[inliers[ppt]],projected3D[inliers[ppt]],cv::Scalar(0,0,255),1);
	}
	for(int ppt=0;ppt<imgPoints.size();ppt++) {
		cv::circle(reprojected, imgPoints[ppt], 2, cv::Scalar(255,0,0), CV_FILLED);
		cv::circle(reprojected, projected3D[ppt], 2, cv::Scalar(0,255,0), CV_FILLED);			
	}
	for (int ppt=0; ppt<inliers.size(); ppt++) {
		cv::circle(reprojected, imgPoints[inliers[ppt]], 2, cv::Scalar(255,255,0), CV_FILLED);
	}
	stringstream ss; ss << "inliers " << inliers.size() << " / " << projected3D.size();
	putText(reprojected, ss.str(), cv::Point(5,20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255), 2);

	cv::imshow("__tmp", reprojected);
	cv::waitKey(0);
	cv::destroyWindow("__tmp");
#endif
	//cv::Rodrigues(rvec, R);
	//visualizerShowCamera(R,t,0,255,0,0.1);

	if(inliers.size() < (double)(imgPoints.size())/5.0) {
		cerr << "not enough inliers to consider a good pose ("<<inliers.size()<<"/"<<imgPoints.size()<<")"<< endl;
		return false;
	}

	if(cv::norm(t) > 200.0) {
		// this is bad...
		cerr << "estimated camera movement is too big, skip this camera\r\n";
		return false;
	}

	cv::Rodrigues(rvec, R);
	if(!CheckCoherentRotation(R)) {
		cerr << "rotation is incoherent. we should try a different base view..." << endl;
		return false;
	}

	std::cout << "found t = " << t << "\nR = \n"<<R<<std::endl;
	return true;
}

bool MultiCameraPnP::TriangulatePointsBetweenViews(
	int working_view, 
	int older_view,
	vector<struct CloudPoint>& new_triangulated,
	vector<int>& add_to_cloud
	) 
{
	cout << " Triangulate " << imgs_names[working_view] << " and " << imgs_names[older_view] << endl;
	//get the left camera matrix
	//TODO: potential bug - the P mat for <view> may not exist? or does it...
	cv::Matx34d P = Pmats[older_view];
	cv::Matx34d P1 = Pmats[working_view];

	std::vector<cv::KeyPoint> pt_set1,pt_set2;
	std::vector<cv::DMatch> matches = matches_matrix[std::make_pair(older_view,working_view)];
	GetAlignedPointsFromMatch(imgpts[older_view],imgpts[working_view],matches,pt_set1,pt_set2);


	//adding more triangulated points to general cloud
	double reproj_error = TriangulatePoints(pt_set1, pt_set2, K, Kinv, distortion_coeff, P, P1, new_triangulated, correspImg1Pt);
	std::cout << "triangulation reproj error " << reproj_error << std::endl;

	vector<uchar> trig_status;
	if(!TestTriangulation(new_triangulated, P, trig_status) || !TestTriangulation(new_triangulated, P1, trig_status)) {
		cerr << "Triangulation did not succeed" << endl;
		return false;
	}
//	if(reproj_error > 20.0) {
//		// somethign went awry, delete those triangulated points
//		//				pcloud.resize(start_i);
//		cerr << "reprojection error too high, don't include these points."<<endl;
//		return false;
//	}

	//filter out outlier points with high reprojection
	vector<double> reprj_errors;
	for(int i=0;i<new_triangulated.size();i++) { reprj_errors.push_back(new_triangulated[i].reprojection_error); }
	std::sort(reprj_errors.begin(),reprj_errors.end());
	//get the 80% precentile
	double reprj_err_cutoff = reprj_errors[4 * reprj_errors.size() / 5] * 2.4; //threshold from Snavely07 4.2
	
	vector<CloudPoint> new_triangulated_filtered;
	std::vector<cv::DMatch> new_matches;
	for(int i=0;i<new_triangulated.size();i++) {
		if(trig_status[i] == 0)
			continue; //point was not in front of camera
		if(new_triangulated[i].reprojection_error > 16.0) {
			continue; //reject point
		} 
		if(new_triangulated[i].reprojection_error < 4.0 ||
			new_triangulated[i].reprojection_error < reprj_err_cutoff) 
		{
			new_triangulated_filtered.push_back(new_triangulated[i]);
			new_matches.push_back(matches[i]);
		} 
		else 
		{
			continue;
		}
	}

	cout << "filtered out " << (new_triangulated.size() - new_triangulated_filtered.size()) << " high-error points" << endl;

	//all points filtered?
	if(new_triangulated_filtered.size() <= 0) return false;
	
	new_triangulated = new_triangulated_filtered;
	
	matches = new_matches;
	matches_matrix[std::make_pair(older_view,working_view)] = new_matches; //just to make sure, remove if unneccesary
	matches_matrix[std::make_pair(working_view,older_view)] = FlipMatches(new_matches);
	add_to_cloud.clear();
	add_to_cloud.resize(new_triangulated.size(),1);
	int found_other_views_count = 0;
	int num_views = imgs.size();

	//scan new triangulated points, if they were already triangulated before - strengthen cloud
	//#pragma omp parallel for num_threads(1)
	for (int j = 0; j<new_triangulated.size(); j++) {
		new_triangulated[j].imgpt_for_img = std::vector<int>(imgs.size(),-1);

		//matches[j] corresponds to new_triangulated[j]
		//matches[j].queryIdx = point in <older_view>
		//matches[j].trainIdx = point in <working_view>
		new_triangulated[j].imgpt_for_img[older_view] = matches[j].queryIdx;	//2D reference to <older_view>
		new_triangulated[j].imgpt_for_img[working_view] = matches[j].trainIdx;		//2D reference to <working_view>
		bool found_in_other_view = false;
		for (unsigned int view_ = 0; view_ < num_views; view_++) {
			if(view_ != older_view) {
				//Look for points in <view_> that match to points in <working_view>
				std::vector<cv::DMatch> submatches = matches_matrix[std::make_pair(view_,working_view)];
				for (unsigned int ii = 0; ii < submatches.size(); ii++) {
					if (submatches[ii].trainIdx == matches[j].trainIdx &&
						!found_in_other_view) 
					{
						//Point was already found in <view_> - strengthen it in the known cloud, if it exists there

						//cout << "2d pt " << submatches[ii].queryIdx << " in img " << view_ << " matched 2d pt " << submatches[ii].trainIdx << " in img " << i << endl;
						for (unsigned int pt3d=0; pt3d<pcloud.size(); pt3d++) {
							if (pcloud[pt3d].imgpt_for_img[view_] == submatches[ii].queryIdx) 
							{
								//pcloud[pt3d] - a point that has 2d reference in <view_>

								//cout << "3d point "<<pt3d<<" in cloud, referenced 2d pt " << submatches[ii].queryIdx << " in view " << view_ << endl;
#pragma omp critical 
								{
									pcloud[pt3d].imgpt_for_img[working_view] = matches[j].trainIdx;
									pcloud[pt3d].imgpt_for_img[older_view] = matches[j].queryIdx;
									found_in_other_view = true;
									add_to_cloud[j] = 0;
								}
							}
						}
					}
				}
			}
		}
#pragma omp critical
		{
			if (found_in_other_view) {
				found_other_views_count++;
			} else {
				add_to_cloud[j] = 1;
			}
		}
	}
	std::cout << found_other_views_count << "/" << new_triangulated.size() << " points were found in other views, adding " << cv::countNonZero(add_to_cloud) << " new\n";
	return true;
}

void MultiCameraPnP::AdjustCurrentBundle() {
	cout << "======================== Bundle Adjustment ==========================\n";

	pointcloud_beforeBA = pcloud;
	GetRGBForPointCloud(pointcloud_beforeBA,pointCloudRGB_beforeBA);
	
	cv::Mat _cam_matrix = K;
	BundleAdjuster BA;
	BA.adjustBundle(pcloud,_cam_matrix,imgpts,Pmats);
	K = cam_matrix;
	Kinv = K.inv();
	
	cout << "use new K " << endl << K << endl;
	
	GetRGBForPointCloud(pcloud,pointCloudRGB);
}	

void MultiCameraPnP::PruneMatchesBasedOnF() {
	//prune the match between <_i> and all views using the Fundamental matrix to prune
//#pragma omp parallel for
	for (int _i=0; _i < imgs.size() - 1; _i++)
	{
		for (unsigned int _j=_i+1; _j < imgs.size(); _j++) {
			int older_view = _i, working_view = _j;

			GetFundamentalMat( imgpts[older_view], 
				imgpts[working_view], 
				imgpts_good[older_view],
				imgpts_good[working_view], 
				matches_matrix[std::make_pair(older_view,working_view)]
#ifdef __SFM__DEBUG__
				,imgs_orig[older_view],imgs_orig[working_view]
#endif
			);
			//update flip matches as well
#pragma omp critical
			matches_matrix[std::make_pair(working_view,older_view)] = FlipMatches(matches_matrix[std::make_pair(older_view,working_view)]);
		}
	}
}

void MultiCameraPnP::RecoverDepthFromImages() {
	if(!features_matched) 
		OnlyMatchFeatures();
	
	std::cout << "======================================================================\n";
	std::cout << "======================== Depth Recovery Start ========================\n";
	std::cout << "======================================================================\n";
	
	PruneMatchesBasedOnF();
	GetBaseLineTriangulation();
	AdjustCurrentBundle();
	update(); //notify listeners
	
	cv::Matx34d P1 = Pmats[m_second_view];
	cv::Mat_<double> t = (cv::Mat_<double>(1,3) << P1(0,3), P1(1,3), P1(2,3));
	cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P1(0,0), P1(0,1), P1(0,2), 
												   P1(1,0), P1(1,1), P1(1,2), 
												   P1(2,0), P1(2,1), P1(2,2));
	cv::Mat_<double> rvec(1,3); Rodrigues(R, rvec);
	
	done_views.insert(m_first_view);
	done_views.insert(m_second_view);
	good_views.insert(m_first_view);
	good_views.insert(m_second_view);

	//loop images to incrementally recover more cameras 
	//for (unsigned int i=0; i < imgs.size(); i++) 
	while (done_views.size() != imgs.size())
	{
		//find image with highest 2d-3d correspondance [Snavely07 4.2]
		unsigned int max_2d3d_view = -1, max_2d3d_count = 0;
		vector<cv::Point3f> max_3d; vector<cv::Point2f> max_2d;
		for (unsigned int _i=0; _i < imgs.size(); _i++) {
			if(done_views.find(_i) != done_views.end()) continue; //already done with this view

			vector<cv::Point3f> tmp3d; vector<cv::Point2f> tmp2d;
			cout << imgs_names[_i] << ": ";
			Find2D3DCorrespondences(_i,tmp3d,tmp2d);
			if(tmp3d.size() > max_2d3d_count) {
				max_2d3d_count = tmp3d.size();
				max_2d3d_view = _i;
				max_3d = tmp3d; max_2d = tmp2d;
			}
		}
		int i = max_2d3d_view; //highest 2d3d matching view

		std::cout << "-------------------------- " << imgs_names[i] << " --------------------------\n";
		done_views.insert(i); // don't repeat it for now

		bool pose_estimated = FindPoseEstimation(i,rvec,t,R,max_3d,max_2d);
		if(!pose_estimated)
			continue;

		//store estimated pose	
		Pmats[i] = cv::Matx34d	(R(0,0),R(0,1),R(0,2),t(0),
								 R(1,0),R(1,1),R(1,2),t(1),
								 R(2,0),R(2,1),R(2,2),t(2));
		
		// start triangulating with previous GOOD views
		for (set<int>::iterator done_view = good_views.begin(); done_view != good_views.end(); ++done_view) 
		{
			int view = *done_view;
			if( view == i ) continue; //skip current...

			cout << " -> " << imgs_names[view] << endl;
			
			vector<CloudPoint> new_triangulated;
			vector<int> add_to_cloud;
			bool good_triangulation = TriangulatePointsBetweenViews(i,view,new_triangulated,add_to_cloud);
			if(!good_triangulation) continue;

			std::cout << "before triangulation: " << pcloud.size();
			for (int j=0; j<add_to_cloud.size(); j++) {
				if(add_to_cloud[j] == 1)
					pcloud.push_back(new_triangulated[j]);
			}
			std::cout << " after " << pcloud.size() << std::endl;
			//break;
		}
		good_views.insert(i);
		
		AdjustCurrentBundle();
		update();
	}

	cout << "======================================================================\n";
	cout << "========================= Depth Recovery DONE ========================\n";
	cout << "======================================================================\n";
}
