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

#include "MultiCameraDistance.h"
#include "Common.h"
#include "SfMUpdateListener.h"

class MultiCameraPnP : public MultiCameraDistance {
	std::vector<CloudPoint> pointcloud_beforeBA;
	std::vector<cv::Vec3b> pointCloudRGB_beforeBA;

public:
	MultiCameraPnP(
		const std::vector<cv::Mat>& imgs_, 
		const std::vector<std::string>& imgs_names_, 
		const std::string& imgs_path_):
	MultiCameraDistance(imgs_,imgs_names_,imgs_path_) 
	{
	}

	virtual void RecoverDepthFromImages();

	std::vector<cv::Point3d> getPointCloudBeforeBA() { return CloudPointsToPoints(pointcloud_beforeBA); }
	const std::vector<cv::Vec3b>& getPointCloudRGBBeforeBA() { return pointCloudRGB_beforeBA; }

private:
	void PruneMatchesBasedOnF();
	void AdjustCurrentBundle();
	void GetBaseLineTriangulation();
	void Find2D3DCorrespondences(int working_view, 
		std::vector<cv::Point3f>& ppcloud, 
		std::vector<cv::Point2f>& imgPoints);
	bool FindPoseEstimation(
		int working_view,
		cv::Mat_<double>& rvec,
		cv::Mat_<double>& t,
		cv::Mat_<double>& R,
		std::vector<cv::Point3f> ppcloud,
		std::vector<cv::Point2f> imgPoints);
	bool TriangulatePointsBetweenViews(
		int working_view, 
		int second_view,
		std::vector<struct CloudPoint>& new_triangulated,
		std::vector<int>& add_to_cloud
		);
	
	int FindHomographyInliers2Views(int vi, int vj);
	int m_first_view;
	int m_second_view; //baseline's second view other to 0
	std::set<int> done_views;
	std::set<int> good_views;
	
/********** Subject / Objserver **********/
	std::vector < SfMUpdateListener * > listeners;
public:
    void attach(SfMUpdateListener *sul)
    {
        listeners.push_back(sul);
    }
private:
    void update()
    {
        for (int i = 0; i < listeners.size(); i++)
			listeners[i]->update(getPointCloud(),
								 getPointCloudRGB(),
								 getPointCloudBeforeBA(),
								 getPointCloudRGBBeforeBA(),
								 getCameras());
    }
};
