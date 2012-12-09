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

#include "BundleAdjuster.h"
#include "Common.h"

#define V3DLIB_ENABLE_SUITESPARSE

#include <Math/v3d_linear.h>
#include <Base/v3d_vrmlio.h>
#include <Geometry/v3d_metricbundle.h>

using namespace V3D;
using namespace std;
using namespace cv; 

namespace
{
	
	inline void
	showErrorStatistics(double const f0,
						StdDistortionFunction const& distortion,
						vector<CameraMatrix> const& cams,
						vector<Vector3d> const& Xs,
						vector<Vector2d> const& measurements,
						vector<int> const& correspondingView,
						vector<int> const& correspondingPoint)
	{
		int const K = measurements.size();
		
		double meanReprojectionError = 0.0;
		for (int k = 0; k < K; ++k)
		{
			int const i = correspondingView[k];
			int const j = correspondingPoint[k];
			Vector2d p = cams[i].projectPoint(distortion, Xs[j]);
			
			double reprojectionError = norm_L2(f0 * (p - measurements[k]));
			meanReprojectionError += reprojectionError;
			
//			cout << "i=" << i << " j=" << j << " k=" << k << "\n";
//			 displayVector(Xs[j]);
//			 displayVector(f0*p);
//			 displayVector(f0*measurements[k]);
//			 displayMatrix(cams[i].getRotation());
//			 displayVector(cams[i].getTranslation());
//			 cout << "##################### error = " << reprojectionError << "\n";
//			 if(reprojectionError > 2)
//             cout << "!\n";
		}
		cout << "mean reprojection error (in pixels): " << meanReprojectionError/K << endl;
	}
} // end namespace <>

//count number of 2D measurements
int BundleAdjuster::Count2DMeasurements(const vector<CloudPoint>& pointcloud) {
	int K = 0;
	for (unsigned int i=0; i<pointcloud.size(); i++) {
		for (unsigned int ii=0; ii<pointcloud[i].imgpt_for_img.size(); ii++) {
			if (pointcloud[i].imgpt_for_img[ii] >= 0) {
				K ++;
			}
		}
	}
	return K;
}

void BundleAdjuster::adjustBundle(vector<CloudPoint>& pointcloud, 
								  Mat& cam_matrix,
								  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
								  std::map<int ,cv::Matx34d>& Pmats
								) 
{
	int N = Pmats.size(), M = pointcloud.size(), K = Count2DMeasurements(pointcloud);
	
	cout << "N (cams) = " << N << " M (points) = " << M << " K (measurements) = " << K << endl;
	
	StdDistortionFunction distortion;
	
	//conver camera intrinsics to BA datastructs
	Matrix3x3d KMat;
	makeIdentityMatrix(KMat);
	KMat[0][0] = cam_matrix.at<double>(0,0); //fx
	KMat[1][1] = cam_matrix.at<double>(1,1); //fy
	KMat[0][1] = cam_matrix.at<double>(0,1); //skew
	KMat[0][2] = cam_matrix.at<double>(0,2); //ppx
	KMat[1][2] = cam_matrix.at<double>(1,2); //ppy
	
	double const f0 = KMat[0][0];
	cout << "intrinsic before bundle = "; displayMatrix(KMat);
	Matrix3x3d Knorm = KMat;
	// Normalize the intrinsic to have unit focal length.
	scaleMatrixIP(1.0/f0, Knorm);
	Knorm[2][2] = 1.0;
	
	vector<int> pointIdFwdMap(M);
	map<int, int> pointIdBwdMap;
	
	//conver 3D point cloud to BA datastructs
	vector<Vector3d > Xs(M);
	for (int j = 0; j < M; ++j)
	{
		int pointId = j;
		Xs[j][0] = pointcloud[j].pt.x;
		Xs[j][1] = pointcloud[j].pt.y;
		Xs[j][2] = pointcloud[j].pt.z;
		pointIdFwdMap[j] = pointId;
		pointIdBwdMap.insert(make_pair(pointId, j));
	}
	cout << "Read the 3D points." << endl;
	
	vector<int> camIdFwdMap(N,-1);
	map<int, int> camIdBwdMap;
	
	//convert cameras to BA datastructs
	vector<CameraMatrix> cams(N);
	for (int i = 0; i < N; ++i)
	{
		int camId = i;
		Matrix3x3d R;
		Vector3d T;
		
		Matx34d& P = Pmats[i];
		
		R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); T[0] = P(0,3);
		R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); T[1] = P(1,3);
		R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); T[2] = P(2,3);
		
		camIdFwdMap[i] = camId;
		camIdBwdMap.insert(make_pair(camId, i));
		
		cams[i].setIntrinsic(Knorm);
		cams[i].setRotation(R);
		cams[i].setTranslation(T);
	}
	cout << "Read the cameras." << endl;
	
	vector<Vector2d > measurements;
	vector<int> correspondingView;
	vector<int> correspondingPoint;
	
	measurements.reserve(K);
	correspondingView.reserve(K);
	correspondingPoint.reserve(K);
	
	//convert 2D measurements to BA datastructs
	for (unsigned int k = 0; k < pointcloud.size(); ++k)
	{
		for (unsigned int i=0; i<pointcloud[k].imgpt_for_img.size(); i++) {
			if (pointcloud[k].imgpt_for_img[i] >= 0) {
				int view = i, point = k;
				Vector3d p, np;
				
				Point cvp = imgpts[i][pointcloud[k].imgpt_for_img[i]].pt;
				p[0] = cvp.x;
				p[1] = cvp.y;
				p[2] = 1.0;
				
				if (camIdBwdMap.find(view) != camIdBwdMap.end() &&
					pointIdBwdMap.find(point) != pointIdBwdMap.end())
				{
					// Normalize the measurements to match the unit focal length.
					scaleVectorIP(1.0/f0, p);
					measurements.push_back(Vector2d(p[0], p[1]));
					correspondingView.push_back(camIdBwdMap[view]);
					correspondingPoint.push_back(pointIdBwdMap[point]);
				}
			}
		}
	} // end for (k)
	
	K = measurements.size();
	
	cout << "Read " << K << " valid 2D measurements." << endl;
	
	showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

//	V3D::optimizerVerbosenessLevel = 1;
	double const inlierThreshold = 2.0 / fabs(f0);
	
	Matrix3x3d K0 = cams[0].getIntrinsic();
	cout << "K0 = "; displayMatrix(K0);

	bool good_adjustment = false;
	{
		ScopedBundleExtrinsicNormalizer extNorm(cams, Xs);
		ScopedBundleIntrinsicNormalizer intNorm(cams,measurements,correspondingView);
		CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_FOCAL_LENGTH_PP, inlierThreshold, K0, distortion, cams, Xs,
												 measurements, correspondingView, correspondingPoint);
//		StdMetricBundleOptimizer opt(inlierThreshold,cams,Xs,measurements,correspondingView,correspondingPoint);
		
		opt.tau = 1e-3;
		opt.maxIterations = 50;
		opt.minimize();
		
		cout << "optimizer status = " << opt.status << endl;
		
		good_adjustment = (opt.status != 2);
	}
	
	cout << "refined K = "; displayMatrix(K0);
	
	for (int i = 0; i < N; ++i) cams[i].setIntrinsic(K0);
	
	Matrix3x3d Knew = K0;
	scaleMatrixIP(f0, Knew);
	Knew[2][2] = 1.0;
	cout << "Knew = "; displayMatrix(Knew);
	
	showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);
	
	if(good_adjustment) { //good adjustment?
		
		//Vector3d mean(0.0, 0.0, 0.0);
		//for (unsigned int j = 0; j < Xs.size(); ++j) addVectorsIP(Xs[j], mean);
		//scaleVectorIP(1.0/Xs.size(), mean);
		//
		//vector<float> norms(Xs.size());
		//for (unsigned int j = 0; j < Xs.size(); ++j)
		//	norms[j] = distance_L2(Xs[j], mean);
		//
		//std::sort(norms.begin(), norms.end());
		//float distThr = norms[int(norms.size() * 0.9f)];
		//cout << "90% quantile distance: " << distThr << endl;
		
		//extract 3D points
		for (unsigned int j = 0; j < Xs.size(); ++j)
		{
			//if (distance_L2(Xs[j], mean) > 3*distThr) makeZeroVector(Xs[j]);
			
			pointcloud[j].pt.x = Xs[j][0];
			pointcloud[j].pt.y = Xs[j][1];
			pointcloud[j].pt.z = Xs[j][2];
		}
		
		//extract adjusted cameras
		for (int i = 0; i < N; ++i)
		{
			Matrix3x3d R = cams[i].getRotation();
			Vector3d T = cams[i].getTranslation();
			
			Matx34d P;
			P(0,0) = R[0][0]; P(0,1) = R[0][1]; P(0,2) = R[0][2]; P(0,3) = T[0];
			P(1,0) = R[1][0]; P(1,1) = R[1][1]; P(1,2) = R[1][2]; P(1,3) = T[1];
			P(2,0) = R[2][0]; P(2,1) = R[2][1]; P(2,2) = R[2][2]; P(2,3) = T[2];
			
			Pmats[i] = P;
		}
		

		//TODO: extract camera intrinsics
		cam_matrix.at<double>(0,0) = Knew[0][0];
		cam_matrix.at<double>(0,1) = Knew[0][1];
		cam_matrix.at<double>(0,2) = Knew[0][2];
		cam_matrix.at<double>(1,1) = Knew[1][1];
		cam_matrix.at<double>(1,2) = Knew[1][2];
	}
}
