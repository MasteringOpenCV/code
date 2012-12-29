#include "PAW.h"
#include "Triangle.h"
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



Mat PAW::getSourceLandmarks(){
    return this->srcLandmarks;
}

Mat PAW::getDestLandmarks(){
    return this->dstLandmarks;
}

int PAW::getNLandmarks(){
    return nLandmarks;
}

int PAW::getBaseImageWidth(){
    return baseImageWidth;
}
int PAW::getBaseImageHeight(){
    return baseImageHeight;
}

map<int,int>& PAW::getMapPointTriangle(){
    return mapPointTriangle;    
}


Mat PAW::getWarp(){
    return warp;
}

void PAW::calculateWarpMatrix(){
    double  x, y, xi, yi, xj, yj, xk, yk, xi0, yi0, xj0, yj0, xk0, yk0;
    double a1,a2,a3,a4,a5,a6;
    int index[3];    
    warp = cv::Mat(nTriangles, 6, CV_64FC1);

    for(int i=0;i<nTriangles;i++){
        index[0] = triangles.at<int>(i,0);
        index[1] = triangles.at<int>(i,1);
        index[2] = triangles.at<int>(i,2);
        xi0 = srcLandmarks.at<int>(index[0],0);
        yi0 = srcLandmarks.at<int>(index[0],1);
        xj0 = srcLandmarks.at<int>(index[1],0);
        yj0 = srcLandmarks.at<int>(index[1],1);
        xk0 = srcLandmarks.at<int>(index[2],0);
        yk0 = srcLandmarks.at<int>(index[2],1);

        xi = dstLandmarks.at<int>(index[0],0);
        yi = dstLandmarks.at<int>(index[0],1);
        xj = dstLandmarks.at<int>(index[1],0);
        yj = dstLandmarks.at<int>(index[1],1);
        xk = dstLandmarks.at<int>(index[2],0);
        yk = dstLandmarks.at<int>(index[2],1);

        a1 = (xi*xj0*yk0 - xi*xk0*yj0 - xi0*xj*yk0 + xi0*xk*yj0 + xj*xk0*yi0 - xj0*xk*yi0)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
        a2 = (xi*yj0 - xj*yi0 - xi*yk0 + xk*yi0 + xj*yk0 - xk*yj0)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
        a3 = -(xi*xj0 - xi0*xj - xi*xk0 + xi0*xk + xj*xk0 - xj0*xk)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);

        a4 = -(xi0*yj*yk0 - xi0*yj0*yk - xj0*yi*yk0 + xj0*yi0*yk + xk0*yi*yj0 - xk0*yi0*yj)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
        a5 = (yi*yj0 - yi0*yj - yi*yk0 + yi0*yk + yj*yk0 - yj0*yk)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);
        a6 = (xi0*yj - xj0*yi - xi0*yk + xk0*yi + xj0*yk - xk0*yj)/(xi0*yj0 - xj0*yi0 - xi0*yk0 + xk0*yi0 + xj0*yk0 - xk0*yj0);

        warp.at<double>(i,0) = a1;
        warp.at<double>(i,1) = a2;
        warp.at<double>(i,2) = a3;
        warp.at<double>(i,3) = a4;
        warp.at<double>(i,4) = a5;
        warp.at<double>(i,5) = a6;

    }
    
}


vector<CvPoint> PAW::getPointsInsideHull(){
    return pointsInsideHull;
}


/*
    This method will calculate the convex hull of source landmarks
    and populate the pointsInsideHull vector with these points coordinates
*/
void PAW::populatePointsInsideHull(){
    //calc scrLandmarks convex hull
    CvPoint* pointsHull = (CvPoint*)malloc( nLandmarks * sizeof(pointsHull[0]));
    int* hull = (int*)malloc( nLandmarks * sizeof(hull[0]));
    CvMat pointMat = cvMat( 1, nLandmarks, CV_32SC2, pointsHull );
    CvMat hullMat = cvMat( 1, nLandmarks, CV_32SC1, hull );
        

    for(int i = 0; i < nLandmarks; i++ )
    {           
        pointsHull[i] = cvPoint(srcLandmarks.at<int>(i,0),srcLandmarks.at<int>(i,1));
    }
    cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
    int hullcount = hullMat.cols;
        
    CvPoint* pointsHullFinal = (CvPoint*)malloc( hullcount * sizeof(pointsHullFinal[0]));        
        
    
    for(int i = 0; i < hullcount; i++ ){
        int ptIndex = hull[i];
        CvPoint pt = cvPoint(    srcLandmarks.at<int>(ptIndex,0),
                                srcLandmarks.at<int>(ptIndex,1));

        pointsHullFinal[i] = pt;
    }

    CvMat hullMatPoints = cvMat( 1, hullcount, CV_32SC2, pointsHullFinal);

    //check if point belongs
    for (int j=0;j<baseImageHeight;j++){            
        for(int i=0;i< baseImageWidth;i++){
            
            double distance = cvPointPolygonTest(&hullMatPoints,cvPoint2D32f(i,j),1);
            if(distance >=0){                    
                pointsInsideHull.push_back(cvPoint(i,j));        
            }
        }
    }
}

/*
    This function uses Delaunay triangulation to populate the
    triangles matrix
*/
void PAW::triangulate(){
    CvMemStorage* storage;
    CvSubdiv2D* subdiv;
    IplImage* img;

    int par;

    std::vector<CvPoint> points;
    vector<int> triangleVertices;

    
    CvRect rect = { 0, 0, baseImageWidth, baseImageHeight};
    storage = cvCreateMemStorage(0);
    subdiv = cvCreateSubdivDelaunay2D(rect,storage);
    

    //insert srcLandmark points in Delaunay subdivision
    for(int i=0;i<nLandmarks;i++){                    
        double x = srcLandmarks.at<int>(i,0);
        double y = srcLandmarks.at<int>(i,1);
        points.push_back(cvPoint(srcLandmarks.at<int>(i,0),srcLandmarks.at<int>(i,1)));
        CvPoint2D32f fp = cvPoint2D32f(x, y);
        cvSubdivDelaunay2DInsert( subdiv, fp );
    }

    

    CvNextEdgeType triangleDirections[2] = {CV_NEXT_AROUND_LEFT,CV_NEXT_AROUND_RIGHT};

    for(int tdi = 0;tdi<2;tdi++){
        CvNextEdgeType triangleDirection = triangleDirections[tdi];

        IplImage* triangleFrame = cvCreateImage(cvSize(baseImageWidth,baseImageHeight),IPL_DEPTH_32F,3);

        CvScalar delaunay_color, voronoi_color;    
        delaunay_color  = CV_RGB( 200,0,0);
        voronoi_color = CV_RGB(0, 200, 0);
    
    
        CvSeqReader  reader;
        int i, total = subdiv->edges->total;
        int elem_size = subdiv->edges->elem_size;

        cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

        CvPoint buf[3];
        printf("Total %d\n",total);
        for( i = 0; i < total; i++ )
        {
            CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

            if( CV_IS_SET_ELEM( edge ))
            {
                //draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );
            
                //TODO optimize this part of code, since we could use a map (and put order) or get points index from delaunay subdiv
                //if(i==par){
                CvSubdiv2DEdge t = (CvSubdiv2DEdge)edge ;
                int shouldPaint=1;
                for(int j=0;j<3;j++){
                    
                    CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
                    if( !pt ) break;
                    buf[j] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));
                    t = cvSubdiv2DGetEdge( t, triangleDirection );
                    if((pt->pt.x<0)||(pt->pt.x>baseImageWidth))
                        shouldPaint=0;
                    if((pt->pt.y<0)||(pt->pt.y>baseImageHeight))
                        shouldPaint=0;
                }
                if(shouldPaint){
                    //cvFillConvexPoly( img, buf, 3, CV_RGB(0,.1+10.0/255.0,0), CV_AA, 0 );
                    int originalVertices[3];
                    for(int j=0;j<3;j++){
                        int px = buf[j].x;
                        int py = buf[j].y;
                        for(int k=0;k<points.size();k++){
                            if((points[k].x ==px) && (points[k].y==py)){
                                printf("%d ",k);
                                originalVertices[j] = k;
                                triangleVertices.push_back(k);
                                break;//could there be overlapped points
                            }
                        }                        
                    }
                    printf("\n");
                    
                    //originalVertices stores the correspondence of vertices 0, 1 and 2 of the currently mapped triangle
                    //with their annotated points (which are in pcaSet)

/*                    int p1x = pcaSet.at<double>(imageIndex,originalVertices[0]*2);
                    int p1y = pcaSet.at<double>(imageIndex,originalVertices[0]*2+1);

                    int p2x = pcaSet.at<double>(imageIndex,originalVertices[1]*2);
                    int p2y = pcaSet.at<double>(imageIndex,originalVertices[1]*2+1);

                    int p3x = pcaSet.at<double>(imageIndex,originalVertices[2]*2);
                    int p3y = pcaSet.at<double>(imageIndex,originalVertices[2]*2+1);

    

                    Point2f srcTri[3];
                    Point2f dstTri[3];
                    
                    srcTri[0] = Point2f( p1x, p1y );
                    srcTri[1] = Point2f( p2x, p2y );
                    srcTri[2] = Point2f( p3x, p3y );
                    
                    dstTri[0] = Point2f( buf[0].x, buf[0].y );
                    dstTri[1] = Point2f( buf[1].x, buf[1].y );
                    dstTri[2] = Point2f( buf[2].x, buf[2].y );
                    */
                    //warpTextureFromTriangle(srcTri, originalImage, dstTri, warp_final);

                    /*cvLine(new IplImage(warp_final),cvPoint(p1x,p1y),cvPoint(p2x,p2y),CV_RGB(0,255,0),1,8,0);
                    cvLine(new IplImage(warp_final),cvPoint(p2x,p2y),cvPoint(p3x,p3y),CV_RGB(0,255,0),1,8,0);
                    cvLine(new IplImage(warp_final),cvPoint(p3x,p3y),cvPoint(p1x,p1y),CV_RGB(0,255,0),1,8,0);*/
                }

            
                //draw_subdiv_edge( triangleFrame, (CvSubdiv2DEdge)edge, delaunay_color );
            }
            CV_NEXT_SEQ_ELEM( elem_size, reader );
        }
    
        //string num = static_cast<ostringstream*>( &(ostringstream() << countFrame++) )->str();
        //imshow("Warped final "+ num,warp_final);

        //clean up repeated triangles

        

        set<Triangle> triangleSet;
        for(int i=0;i<triangleVertices.size()/3;i+=1){
            printf("%2d %2d %2d\n",triangleVertices.at(3*i),triangleVertices.at(3*i+1),triangleVertices.at(3*i+2));
            Triangle t(triangleVertices.at(3*i),triangleVertices.at(3*i+1),triangleVertices.at(3*i+2));
            triangleSet.insert(t);
        }


        triangles = Mat::zeros(triangleSet.size(),3,CV_32S);

        set<Triangle>::iterator it;
        int count=0;
        for (it=triangleSet.begin(); it!=triangleSet.end(); it++){
            cout << (*it).v1 << " " << (*it).v2 << " " << (*it).v3 << endl;
            triangles.at<int>(count,0) = ( (*it).v1);
            triangles.at<int>(count,1) = ( (*it).v2);
            triangles.at<int>(count,2) = ( (*it).v3);
            count++;
        }
        cout << endl;
        nTriangles = count;

        Mat triangleMat(triangleFrame);
        imshow("Triangle frame",triangleMat);

        populatePointTriangleMap();
    }

}

bool PAW::isPointInsideTriangleIndex(int px, int py, int triangleIndex){
    //look for triangles in source landmarks
    int v1, v2, v3;
    v1 = triangles.at<int>(triangleIndex,0);
    v2 = triangles.at<int>(triangleIndex,1);
    v3 = triangles.at<int>(triangleIndex,2);

    int x1 = srcLandmarks.at<int>(v1,0);
    int y1 = srcLandmarks.at<int>(v1,1);
    int x2 = srcLandmarks.at<int>(v2,0);
    int y2 = srcLandmarks.at<int>(v2,1);
    int x3 = srcLandmarks.at<int>(v3,0);
    int y3 = srcLandmarks.at<int>(v3,1);

    return isPointInsideTriangle(px,py,x1,y1,x2,y2,x3,y3);
}

//uses barycentric coordinates from wikipedia
bool PAW::isPointInsideTriangle(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3){
    double denominator = (y2-y3)*(x1-x3)+(x3-x2)*(y1-y3);
    double lambda1 = ((y2-y3)*(x-x3)+(x3-x2)*(y-y3))/denominator;
    double lambda2 = ((y3-y1)*(x-x3)+(x1-x3)*(y-y3))/denominator;
    double lambda3 = 1 - lambda1 - lambda2;
    double eps = 0.000000001;
    bool belongs = false;
    if((0-eps <= lambda1)&&(lambda1 <=1+eps)){
        if((0-eps <= lambda2)&&(lambda2 <=1+eps)){
            if((0-eps <= lambda3)&&(lambda3 <=1+eps)){
                belongs = true;
            }
        }
    }
    return belongs;
}

void PAW::populatePointTriangleMap(){
    //-1 stands for no triangle
    for(int i=0;i<baseImageHeight;i++){
        for(int j=0;j<baseImageWidth;j++){
            int index = -1;
            for(int k=0;k<nTriangles;k++){
                if(isPointInsideTriangleIndex(j,i,k)){
                    index = k;
                    break;
                }
            }            
            mapPointTriangle[i*baseImageWidth+j]=index;
//            cout << index ;
        }
//        cout << endl;
    }
}


Mat PAW::getTriangles(){
    return triangles;
}

void PAW::init(){    
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
