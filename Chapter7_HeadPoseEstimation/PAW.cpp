#include "PAW.h"
#include "Triangle.h"
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <algorithm>
#include <set>

PAW::PAW(Mat srcLandmarks, Mat dstLandmarks, int width, int height)
{
	this->srcLandmarks = srcLandmarks;
	this->dstLandmarks = dstLandmarks;
	this->baseImageWidth = width;
	this->baseImageHeight = height;
	nLandmarks = srcLandmarks.rows;
	init();
}



Mat PAW::getSourceLandmarks() {
	return this->srcLandmarks;
}

Mat PAW::getDestLandmarks() {
	return this->dstLandmarks;
}

int PAW::getNLandmarks() {
	return nLandmarks;
}

int PAW::getBaseImageWidth() {
	return baseImageWidth;
}
int PAW::getBaseImageHeight() {
	return baseImageHeight;
}

map<int, int>& PAW::getMapPointTriangle() {
	return mapPointTriangle;
}


Mat PAW::getWarp() {
	return warp;
}

void PAW::calculateWarpMatrix() {
	double  x, y, xi, yi, xj, yj, xk, yk, xi0, yi0, xj0, yj0, xk0, yk0;
	double a1, a2, a3, a4, a5, a6;
	int index[3];
	warp = cv::Mat(nTriangles, 6, CV_64FC1);

	for (int i = 0;i < nTriangles;i++) {
		index[0] = triangles.at<int>(i, 0);
		index[1] = triangles.at<int>(i, 1);
		index[2] = triangles.at<int>(i, 2);
		xi0 = srcLandmarks.at<int>(index[0], 0);
		yi0 = srcLandmarks.at<int>(index[0], 1);
		xj0 = srcLandmarks.at<int>(index[1], 0);
		yj0 = srcLandmarks.at<int>(index[1], 1);
		xk0 = srcLandmarks.at<int>(index[2], 0);
		yk0 = srcLandmarks.at<int>(index[2], 1);

		xi = dstLandmarks.at<int>(index[0], 0);
		yi = dstLandmarks.at<int>(index[0], 1);
		xj = dstLandmarks.at<int>(index[1], 0);
		yj = dstLandmarks.at<int>(index[1], 1);
		xk = dstLandmarks.at<int>(index[2], 0);
		yk = dstLandmarks.at<int>(index[2], 1);

		a1 = (xi*xj0*yk0 - xi*xk0*yj0 - xi0*xj*yk0 + xi0*xk*yj0 + xj*xk0*yi0 - xj0*xk*yi0) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
		a2 = (xi*yj0 - xj*yi0 - xi*yk0 + xk*yi0 + xj*yk0 - xk*yj0) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
		a3 = -(xi*xj0 - xi0*xj - xi*xk0 + xi0*xk + xj*xk0 - xj0*xk) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);

		a4 = -(xi0*yj*yk0 - xi0*yj0*yk - xj0*yi*yk0 + xj0*yi0*yk + xk0*yi*yj0 - xk0*yi0*yj) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
		a5 = (yi*yj0 - yi0*yj - yi*yk0 + yi0*yk + yj*yk0 - yj0*yk) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
		a6 = (xi0*yj - xj0*yi - xi0*yk + xk0*yi + xj0*yk - xk0*yj) / (xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);

		warp.at<double>(i, 0) = a1;
		warp.at<double>(i, 1) = a2;
		warp.at<double>(i, 2) = a3;
		warp.at<double>(i, 3) = a4;
		warp.at<double>(i, 4) = a5;
		warp.at<double>(i, 5) = a6;

	}

}


vector<CvPoint> PAW::getPointsInsideHull() {
	return pointsInsideHull;
}


/*
	This method will calculate the convex hull of source landmarks
	and populate the pointsInsideHull vector with these points coordinates
*/
void PAW::populatePointsInsideHull() {
	//calc scrLandmarks convex hull
	CvPoint* pointsHull = (CvPoint*)malloc(nLandmarks * sizeof(pointsHull[0]));
	int* hull = (int*)malloc(nLandmarks * sizeof(hull[0]));
	CvMat pointMat = cvMat(1, nLandmarks, CV_32SC2, pointsHull);
	CvMat hullMat = cvMat(1, nLandmarks, CV_32SC1, hull);


	for (int i = 0; i < nLandmarks; i++)
	{
		pointsHull[i] = cvPoint(srcLandmarks.at<int>(i, 0), srcLandmarks.at<int>(i, 1));
	}
	cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);
	int hullcount = hullMat.cols;

	CvPoint* pointsHullFinal = (CvPoint*)malloc(hullcount * sizeof(pointsHullFinal[0]));


	for (int i = 0; i < hullcount; i++) {
		int ptIndex = hull[i];
		CvPoint pt = cvPoint(srcLandmarks.at<int>(ptIndex, 0),
			srcLandmarks.at<int>(ptIndex, 1));

		pointsHullFinal[i] = pt;
	}

	CvMat hullMatPoints = cvMat(1, hullcount, CV_32SC2, pointsHullFinal);

	//check if point belongs
	for (int j = 0;j < baseImageHeight;j++) {
		for (int i = 0;i < baseImageWidth;i++) {

			double distance = cvPointPolygonTest(&hullMatPoints, cvPoint2D32f(i, j), 1);
			if (distance >= 0) {
				pointsInsideHull.push_back(cvPoint(i, j));
			}
		}
	}
}

/*
	This function uses Delaunay triangulation to populate the
	triangles matrix
*/
void PAW::triangulate() {
	//Opencv3 CvMemStorage* storage;
	Subdiv2D* subdiv;
	IplImage* img;

	int par;

	std::vector<CvPoint> points;
	vector<int> triangleVertices;


	CvRect rect = { 0, 0, baseImageWidth, baseImageHeight };
	//Opencv3 storage = cvCreateMemStorage(0);
	//Opencv3 subdiv = cvCreateSubdivDelaunay2D(rect,storage);

	subdiv = new Subdiv2D(rect);


	//insert srcLandmark points in Delaunay subdivision
	for (int i = 0;i < nLandmarks;i++) {
		double x = srcLandmarks.at<int>(i, 0);
		double y = srcLandmarks.at<int>(i, 1);
		points.push_back(cvPoint(srcLandmarks.at<int>(i, 0), srcLandmarks.at<int>(i, 1)));
		CvPoint2D32f fp = cvPoint2D32f(x, y);
		//Opencv3 cvSubdivDelaunay2DInsert( subdiv, fp );
		subdiv->insert(fp);
	}




	//clean up repeated triangles



	set<Triangle> triangleSet;
	for (int i = 0;i < triangleVertices.size() / 3;i += 1) {
		printf("%2d %2d %2d\n", triangleVertices.at(3 * i), triangleVertices.at(3 * i + 1), triangleVertices.at(3 * i + 2));
		Triangle t(triangleVertices.at(3 * i), triangleVertices.at(3 * i + 1), triangleVertices.at(3 * i + 2));
		triangleSet.insert(t);
	}


	triangles = Mat::zeros(triangleSet.size(), 3, CV_32S);

	set<Triangle>::iterator it;
	int count = 0;
	for (it = triangleSet.begin(); it != triangleSet.end(); it++) {
		cout << (*it).v1 << " " << (*it).v2 << " " << (*it).v3 << endl;
		triangles.at<int>(count, 0) = ((*it).v1);
		triangles.at<int>(count, 1) = ((*it).v2);
		triangles.at<int>(count, 2) = ((*it).v3);
		count++;
	}
	cout << endl;
	nTriangles = count;


    vector<Vec6f> triangleList;
	
    subdiv->getTriangleList(triangleList);
    vector<Point> pt(3);

	
	Mat triangleMat(0, 0, baseImageWidth, baseImageHeight);
	Scalar delaunay_color(255, 255, 255);
    for( size_t i = 0; i < triangleList.size(); i++ )
    {
        Vec6f t = triangleList[i];
        pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
        line(triangleMat, pt[0], pt[1], delaunay_color, 1, LINE_AA, 0);
        line(triangleMat, pt[1], pt[2], delaunay_color, 1, LINE_AA, 0);
        line(triangleMat, pt[2], pt[0], delaunay_color, 1, LINE_AA, 0);
    }


// end


	
	imshow("Triangle frame", triangleMat);

	populatePointTriangleMap();


}

bool PAW::isPointInsideTriangleIndex(int px, int py, int triangleIndex) {
	//look for triangles in source landmarks
	int v1, v2, v3;
	v1 = triangles.at<int>(triangleIndex, 0);
	v2 = triangles.at<int>(triangleIndex, 1);
	v3 = triangles.at<int>(triangleIndex, 2);

	int x1 = srcLandmarks.at<int>(v1, 0);
	int y1 = srcLandmarks.at<int>(v1, 1);
	int x2 = srcLandmarks.at<int>(v2, 0);
	int y2 = srcLandmarks.at<int>(v2, 1);
	int x3 = srcLandmarks.at<int>(v3, 0);
	int y3 = srcLandmarks.at<int>(v3, 1);

	return isPointInsideTriangle(px, py, x1, y1, x2, y2, x3, y3);
}

//uses barycentric coordinates from wikipedia
bool PAW::isPointInsideTriangle(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3) {
	double denominator = (y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3);
	double lambda1 = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / denominator;
	double lambda2 = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / denominator;
	double lambda3 = 1 - lambda1 - lambda2;
	double eps = 0.000000001;
	bool belongs = false;
	if ((0 - eps <= lambda1) && (lambda1 <= 1 + eps)) {
		if ((0 - eps <= lambda2) && (lambda2 <= 1 + eps)) {
			if ((0 - eps <= lambda3) && (lambda3 <= 1 + eps)) {
				belongs = true;
			}
		}
	}
	return belongs;
}

void PAW::populatePointTriangleMap() {
	//-1 stands for no triangle
	for (int i = 0;i < baseImageHeight;i++) {
		for (int j = 0;j < baseImageWidth;j++) {
			int index = -1;
			for (int k = 0;k < nTriangles;k++) {
				if (isPointInsideTriangleIndex(j, i, k)) {
					index = k;
					break;
				}
			}
			mapPointTriangle[i*baseImageWidth + j] = index;
			//            cout << index ;
		}
		//        cout << endl;
	}
}


Mat PAW::getTriangles() {
	return triangles;
}

void PAW::init() {
	//depends on source landmarks
	printf("Points inside hull\n");
	populatePointsInsideHull();

	printf("Triangulate\n");
	triangulate();

	//depends on dest landmarks
	printf("Warp equations\n");
	calculateWarpMatrix();

}




PAW::~PAW(void)
{
}
