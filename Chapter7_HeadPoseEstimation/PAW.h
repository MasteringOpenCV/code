#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace cv;

class PAW
{
public:
    PAW(Mat srcLandmarks, Mat dstLandmarks,int width, int height);
    vector<CvPoint> getPointsInsideHull();
    Mat getTriangles();
    Mat getWarp();
    int getNLandmarks();
    int getBaseImageWidth();
    int getBaseImageHeight();
    Mat getSourceLandmarks();
    Mat getDestLandmarks();
    map<int,int>& getMapPointTriangle();

    ~PAW(void);
private:
    int nLandmarks;
    int nPixels;
    int nTriangles;
    
    //width and height of base image to be swept
    //this image contains the base mesh
    int baseImageWidth, baseImageHeight; 

    map<int,int> mapPointTriangle;
    
    
    //src and dst landmark matrix follow the format
    // [x0 y0
    //  x1 y1
    //  x2 y2
    //  x3 y3]]
    Mat srcLandmarks;
    Mat dstLandmarks;

    // nx3 integer matrix with triangle indexes
    // 0 1 2
    // 0 2 3
    // ...
    
    Mat triangles;


    // warp matrix is in the following format
    // a10 a20 a30 a40 a50 a60
    // a11 a21 a31 a41 a51 a61
    // a12 a22 a32 a42 a52 a62
    Mat warp;


    // alphas
    // [alpha0
    //  alpha1
    //  ...
    //  alphaTriangleN]
    Mat alphas, betas;

    vector<CvPoint> pointsInsideHull;

    //calculates convexHull, nPixels and triangles
    void init();
    void populatePointsInsideHull();
    void triangulate();
    void calculateWarpMatrix();    
    bool isPointInsideTriangleIndex(int px, int py, int triangleIndex);
    bool isPointInsideTriangle(int px, int py, int t0x, int t0y, int t1x, int t1y, int t2x, int t2y);
    void populatePointTriangleMap();


};

