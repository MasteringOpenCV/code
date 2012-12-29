/*****************************************************************************
*   3D Head Pose Estimation using AAM and POSIT
******************************************************************************
*   by Daniel Lélis Baggio, 29th Dec 2012
*   http://code.google.com/p/ehci/
******************************************************************************
*   Ch7 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

/*#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>*/
#include <opencv2/opencv.hpp>
#include "PAW.h"
#include "Triangle.h"

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <algorithm>
#include <set>

using namespace cv;
using namespace std;


void warpTextureFromTriangle(Point2f srcTri[3], Mat originalImage, Point2f dstTri[3], Mat warp_final){
    //int t, ellap;
    //t= clock();
    Mat warp_mat( 2, 3, CV_32FC1 );
    Mat warp_dst, warp_mask;
    CvPoint trianglePoints[3];
    trianglePoints[0] = dstTri[0];
    trianglePoints[1] = dstTri[1];
    trianglePoints[2] = dstTri[2];
    warp_dst  = Mat::zeros( originalImage.rows, originalImage.cols, originalImage.type() );
    warp_mask = Mat::zeros( originalImage.rows, originalImage.cols, originalImage.type() );

    /// Get the Affine Transform
    for(int i=0;i<3;i++){
        srcTri[i].x -= 190;
        srcTri[i].y -= 250;
        dstTri[i].x -=190;
        dstTri[i].y -=250;

    }
    
    warp_mat = getAffineTransform( srcTri, dstTri );
    

    /// Apply the Affine Transform just found to the src image
    Rect roi(190, 250, 240,150);
    Mat originalImageRoi= originalImage(roi);
    Mat warp_dstRoi     = warp_dst(roi);
    warpAffine( originalImageRoi, warp_dstRoi, warp_mat, warp_dstRoi.size() );
    cvFillConvexPoly( new IplImage(warp_mask), trianglePoints, 3, CV_RGB(255,255,255), CV_AA, 0 );    
    warp_dst.copyTo(warp_final,warp_mask);
    
}

PCA loadPCA(const char* fileName, int& rows, int& cols,Mat& pcaset){
    FILE* in = fopen(fileName,"r");
    int a;
    fscanf(in,"%d%d",&rows,&cols);

    pcaset = Mat::eye(rows,cols,CV_64F);
    int i,j;
    i=j=0;
      
    for(i=0;i<rows;i++){
        for(j=0;j<cols;j++){
            fscanf(in,"%d",&a);
            pcaset.at<double>(i,j) = a;
        }
    }
    cout << pcaset << endl;

    PCA pca(pcaset, // pass the data
        Mat(), // we do not have a pre-computed mean vector,
        // so let the PCA engine to compute it
        CV_PCA_DATA_AS_ROW, // indicate that the vectors
        // are stored as matrix rows
        // (use CV_PCA_DATA_AS_COL if the vectors are
        // the matrix columns)
        pcaset.cols// specify, how many principal components to retain
        );
    return pca;
}

void drawPoints(Mat pcaset, PCA pca, PCA pcaTexture, std::vector<CvPoint>& pointsInsideHull,
    std::vector<int> triangleIndexes){
    
    
    int value1 = 50;
    int value2 = 50;
    int value3 = 50;
    int alphaMax = 100;
    int imageCount =1;

    char imageFileName[100];
    sprintf(imageFileName,"09-%dm.jpg",imageCount);
    
    Mat coeffs = Mat::zeros (1,3, CV_64F);
    
    namedWindow("AAM");
    
    IplImage* img = cvLoadImage(imageFileName);
    Mat imageFrame(img);

    int t, ellap;
    
    while(1){
        t = clock();

        sprintf(imageFileName,"09-%dm.jpg",imageCount);
        
        img = cvLoadImage(imageFileName);
        Mat image(img);
        
        createTrackbar("eigen1", "AAM", &value1, alphaMax);
        createTrackbar("eigen2", "AAM", &value2, alphaMax);
        createTrackbar("eigen3", "AAM", &value3, alphaMax);
        
        //Mat image = Mat::zeros (480,640, CV_8UC3);//(240,320, CV_8UC3);

        coeffs.at<double>(0,0) = ((value1*1.0/alphaMax)-0.5)*2 *3* sqrt(pca.eigenvalues.at<double>(0,0));//.at<double>(0,0);
        coeffs.at<double>(0,1) = ((value2*1.0/alphaMax)-0.5)*2 *3* sqrt(pca.eigenvalues.at<double>(1,0));
        coeffs.at<double>(0,2) = ((value3*1.0/alphaMax)-0.5)*2 *3* sqrt(pca.eigenvalues.at<double>(2,0));
        
        Mat back;
        Mat backTexture;
        Mat aamTexture =  Mat::zeros (480,640, imageFrame.type());
        
        pca.backProject(coeffs,back);
        pcaTexture.backProject(coeffs,backTexture);
        

        for(int i=0;i<pointsInsideHull.size();i++){
        
            double v1 = ((backTexture.at<double>(0,3*i+0))*255);
            double v2 = ((backTexture.at<double>(0,3*i+1))*255);
            double v3 = ((backTexture.at<double>(0,3*i+2))*255);
            v1 = (v1> 255) ? 255 : v1;
            v2 = (v2> 255) ? 255 : v2;
            v3 = (v3> 255) ? 255 : v3;

            v1 = (v1< 0) ? 0 : v1;
            v2 = (v2< 0) ? 0 : v2;
            v3 = (v3< 0) ? 0 : v3;            
            
            aamTexture.at<Vec3b>(pointsInsideHull.at(i))[0] = v1;
            aamTexture.at<Vec3b>(pointsInsideHull.at(i))[1] = v2;
            aamTexture.at<Vec3b>(pointsInsideHull.at(i))[2] = v3;
               
        }
        imshow("AAM Texture",aamTexture);

        //draw aam
        for(int j=0;j<(back.cols/2)-1;j++){

            Point2f p (back.at<double>(0,2*j), back.at<double>(0,2*j+1));
            Point2f p2 (back.at<double>(0,2*j+2), back.at<double>(0,2*j+3));
            line(image,p,p2,CV_RGB(255,0,0),3,8,0);
            circle( image, p, 4, CV_RGB(128,0,0), -1, 8);
            circle( image, p2, 4, CV_RGB(128,0,0), -1, 8);
                
        }        
                        
        //warp texture        

        for(int i=0;i<triangleIndexes.size()/3;i++){
            Point2f sourcePoints[3];
            Point2f destPoints[3];
            for(int j=0;j<3;j++){
                int index = triangleIndexes.at(3*i+j);
                
                sourcePoints[j].x = pca.mean.at<double>(0,2*index);
                sourcePoints[j].y = pca.mean.at<double>(0,2*index+1);
                destPoints[j].x   = back.at<double>(0,2*index);
                destPoints[j].y   = back.at<double>(0,2*index+1);
            }            
            warpTextureFromTriangle(sourcePoints, aamTexture, destPoints, image);
        }
        ellap = clock();        
        t = clock();

        image.copyTo(imageFrame);
        imshow("AAM",imageFrame);
        char c = waitKey(10);
        if(c=='c') break;
        if(c=='1') imageCount=1;
        if(c=='2') imageCount=2;
        if(c=='3') imageCount=3;

        cvReleaseImage(&img);
        ellap = clock();        
    }


}


CvSubdiv2D* init_delaunay( CvMemStorage* storage,
                           CvRect rect )
{
    CvSubdiv2D* subdiv;

    subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
                               sizeof(CvSubdiv2DPoint),
                               sizeof(CvQuadEdge2D),
                               storage );
    cvInitSubdivDelaunay2D( subdiv, rect );

    return subdiv;
}

void draw_subdiv_edge( IplImage* img, CvSubdiv2DEdge edge, CvScalar color )
{
    CvSubdiv2DPoint* org_pt;
    CvSubdiv2DPoint* dst_pt;
    CvPoint2D32f org;
    CvPoint2D32f dst;
    CvPoint iorg, idst;

    org_pt = cvSubdiv2DEdgeOrg(edge);
    dst_pt = cvSubdiv2DEdgeDst(edge);

    if( org_pt && dst_pt )
    {
        org = org_pt->pt;
        dst = dst_pt->pt;

        iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
        idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));

        cvLine( img, iorg, idst, color, 1, CV_AA, 0 );
        
    }
    
}

int countFrame=0;
void draw_subdiv( IplImage* img, CvSubdiv2D* subdiv,int par,CvNextEdgeType triangleDirection, std::vector<CvPoint> points,Mat pcaSet,Mat originalImage, int imageIndex, Mat& warp_final, vector<int>& triangleVertices)                
{
    IplImage* triangleFrame = cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,3);

    CvScalar delaunay_color, voronoi_color;    
    delaunay_color  = CV_RGB( 200,0,0);
    voronoi_color = CV_RGB(0, 200, 0);
    
    
    CvSeqReader  reader;
    int i, total = subdiv->edges->total;
    int elem_size = subdiv->edges->elem_size;

    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

    CvPoint buf[3];
    
    for( i = 0; i < total; i++ )
    {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

        if( CV_IS_SET_ELEM( edge ))
        {
            //draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );
            CvSubdiv2DEdge t = (CvSubdiv2DEdge)edge ;
            int shouldPaint=1;
            for(int j=0;j<3;j++){
                    
                CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
                if( !pt ) break;
                buf[j] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));
                t = cvSubdiv2DGetEdge( t, triangleDirection );
                if((pt->pt.x<0)||(pt->pt.x>640))
                    shouldPaint=0;
                if((pt->pt.y<0)||(pt->pt.y>480))
                    shouldPaint=0;
            }
            if(shouldPaint){
                cvFillConvexPoly( img, buf, 3, CV_RGB(0,.1+par*10.0/255.0,0), CV_AA, 0 );
                int originalVertices[3];
                for(int j=0;j<3;j++){
                    int px = buf[j].x;
                    int py = buf[j].y;
                    for(int k=0;k<points.size();k++){
                        if((points[k].x ==px) && (points[k].y==py)){                            
                            originalVertices[j] = k;
                            triangleVertices.push_back(k);
                            break;//could there be overlapped points
                        }
                    }                        
                }
                
                    
                //originalVertices stores the correspondence of vertices 0, 1 and 2 of the currently mapped triangle
                //with their annotated points (which are in pcaSet)

                int p1x = pcaSet.at<double>(imageIndex,originalVertices[0]*2);
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

                warpTextureFromTriangle(srcTri, originalImage, dstTri, warp_final);           
            }

            
            draw_subdiv_edge( triangleFrame, (CvSubdiv2DEdge)edge, delaunay_color );
        }
        CV_NEXT_SEQ_ELEM( elem_size, reader );
    }
    
    string num = static_cast<ostringstream*>( &(ostringstream() << countFrame++) )->str();    
    Mat triangleMat(triangleFrame);
    imshow("Triangle frame",triangleMat);
}

void createAAM(PCA pca, Mat pcaSet, PCA& pcaTexture, std::vector<CvPoint>& pointsInsideHull,vector<int>& triangleVertices){
    CvMemStorage* storage;
    CvSubdiv2D* subdiv;
    CvRect rect = { 0, 0, 640, 480 };
    IplImage* asmFrame = cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,3); //TODO parameterize size
    storage = cvCreateMemStorage(0);
    subdiv = cvCreateSubdivDelaunay2D(rect,storage);//init_delaunay( storage, rect );
    std::vector<CvPoint> points;

    for(int i=0;i<pca.mean.cols/2;i++){        
        double x = pca.mean.at<double>(0,2*i);
        double y = pca.mean.at<double>(0,2*i+1);
        CvPoint point = cvPoint( cvRound(x), cvRound(y));
        points.push_back(point);        
        CvPoint2D32f fp = cvPoint2D32f(x, y);
        cvSubdivDelaunay2DInsert( subdiv, fp );
    }

   
    //create convex hull
    
    CvPoint* pointsHull = (CvPoint*)malloc( points.size() * sizeof(pointsHull[0]));
    int* hull = (int*)malloc( points.size() * sizeof(hull[0]));
    CvMat pointMat = cvMat( 1, points.size(), CV_32SC2, pointsHull );
    CvMat hullMat = cvMat( 1, points.size(), CV_32SC1, hull );
        

    for(int i = 0; i < points.size(); i++ )
    {           
        pointsHull[i] = points.at(i);
    }
    cvConvexHull2( &pointMat, &hullMat, CV_CLOCKWISE, 0 );
    int hullcount = hullMat.cols;
        
    CvPoint* pointsHullFinal = (CvPoint*)malloc( hullcount * sizeof(pointsHullFinal[0]));        
        
    CvPoint pt0 = points[hull[hullcount-1]];

    for(int i = 0; i < hullcount; i++ ){
        CvPoint pt = points[hull[i]];        
        pt0 = pt;
        pointsHullFinal[i] = pt0;
    }

    CvMat hullMatPoints = cvMat( 1, hullcount, CV_32SC2, pointsHullFinal);

    //check if point belongs
    for(int i=0;i< 640;i++){
        for (int j=0;j<480;j++){                
            double distance = cvPointPolygonTest(&hullMatPoints,cvPoint2D32f(i,j),1);
            if(distance >=0){                    
                pointsInsideHull.push_back(cvPoint(i,j));        
            }
        }
    }
    
    
    
    int textureRows = pcaSet.rows;
    int textureCols = pointsInsideHull.size();
    Mat pcaTextureSet = Mat::eye(textureRows,textureCols*3,CV_64F);
    //Mat pcaTextureSet = Mat::eye(textureRows,textureCols,CV_32FC3);
    

    for(int imageIndex=0;imageIndex<3;imageIndex++){

        char imageFileName[200];
        sprintf(imageFileName,"09-%dm.jpg",imageIndex+1);
        IplImage* img = cvLoadImage(imageFileName);
        Mat matImgFrame(img);        

        Mat warp_final;
        warp_final = Mat::zeros( matImgFrame.rows, matImgFrame.cols, matImgFrame.type() );



        draw_subdiv(asmFrame,subdiv,10,CV_NEXT_AROUND_LEFT,points,pcaSet,matImgFrame,imageIndex,warp_final,triangleVertices);
        draw_subdiv(asmFrame,subdiv,10,CV_NEXT_AROUND_RIGHT,points,pcaSet,matImgFrame,imageIndex,warp_final,triangleVertices);
    
        
        
        
        int pointIndex = 0;

        
        for(int j=0;j<textureCols;j++){    
            CvPoint pt = pointsInsideHull.at(pointIndex);            
            int pos = pt.y* img->widthStep + pt.x *3;


            pcaTextureSet.at<double>(imageIndex,3*j  ) = ((double)*((uchar*)(warp_final.data  + pos)))/255.0f;
            pcaTextureSet.at<double>(imageIndex,3*j+1) = ((double)*((uchar*)(warp_final.data  + pos+1)))/255.0f;
            pcaTextureSet.at<double>(imageIndex,3*j+2) = ((double)*((uchar*)(warp_final.data  + pos+2)))/255.0f;
            pointIndex++;
        }

        cvReleaseImage(&img);
        warp_final.release();
        matImgFrame.release();
        
        
    }

    pcaTexture= PCA(pcaTextureSet, // pass the data
        Mat(), // we do not have a pre-computed mean vector,
        // so let the PCA engine to compute it
        CV_PCA_DATA_AS_ROW, // indicate that the vectors
        // are stored as matrix rows
        // (use CV_PCA_DATA_AS_COL if the vectors are
        // the matrix columns)
        pcaTextureSet.cols// specify, how many principal components to retain
        );
     
}



void testMain( int argc, char** argv ){
    Mat pcaset;
    int rows,cols;
    PCA pcaShape = loadPCA("simple-aam.txt",rows,cols,pcaset);


    cout << "Result: eigenvalues" << endl;
    cout << pcaShape.eigenvalues << endl;

    cout << "Result: eigenvectors" << endl;
    cout << pcaShape.eigenvectors << endl;

    cout << "Result: mean" << endl;
    cout << pcaShape.mean << endl;

    for(int i = 0; i < rows; i++ )
    {
        Mat vec = pcaset.row(i), coeffs;
        // compress the vector, the result will be stored
        // in the i-th row of the output matrix
        pcaShape.project(vec, coeffs);
        // and then reconstruct it
        cout << coeffs << endl;
        Mat back;
        pcaShape.backProject(coeffs,back);
        cout << back << endl << endl;

    }

    //asm related
    PCA pcaTexture;
    std::vector<CvPoint> pointsInsideHull;
    std::vector<int>     triangleIndexes;
    std::vector<int>     triangleUnrepeat;
    createAAM(pcaShape,pcaset,pcaTexture, pointsInsideHull,triangleIndexes);
    
 
    set<Triangle> triangles;
    for(int i=0;i<triangleIndexes.size()/3;i+=1){        
        Triangle t(triangleIndexes.at(3*i),triangleIndexes.at(3*i+1),triangleIndexes.at(3*i+2));
        triangles.insert(t);
    }
    
    set<Triangle>::iterator it;
    for (it=triangles.begin(); it!=triangles.end(); it++){
        triangleUnrepeat.push_back( (*it).v1);
        triangleUnrepeat.push_back( (*it).v2);
        triangleUnrepeat.push_back( (*it).v3);
    }
    cout << endl;
    drawPoints(pcaset,pcaShape, pcaTexture, pointsInsideHull,triangleUnrepeat);
}


int main( int argc, char** argv ){
    testMain(argc,argv);
    return 0;
}
