/*****************************************************************************
*   Non-Rigid Face Tracking
******************************************************************************
*   by Jason Saragih, 5th Dec 2012
*   http://jsaragih.org/
******************************************************************************
*   Ch6 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/
/*
  visualize_shape_model: animate the modes of variation in a shape_model object
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"      // For 'sprintf()'
//==============================================================================
void
draw_string(Mat img,                       //image to draw on
        const string text)             //text to draw
{
  Size size = getTextSize(text,FONT_HERSHEY_COMPLEX,0.6f,1,NULL);
  putText(img,text,Point(0,size.height),FONT_HERSHEY_COMPLEX,0.6f,
      Scalar::all(0),1,CV_AA);
  putText(img,text,Point(1,size.height+1),FONT_HERSHEY_COMPLEX,0.6f,
      Scalar::all(255),1,CV_AA);
}
//==============================================================================
void
draw_shape(Mat &img,
       const vector<Point2f> &q,
       const Mat &C)
{
  int n = q.size();
  for(int i = 0; i < C.rows; i++)
    line(img,q[C.at<int>(i,0)],q[C.at<int>(i,1)],CV_RGB(255,0,0),2);
  for(int i = 0; i < n; i++)
    circle(img,q[i],1,CV_RGB(0,0,0),2,CV_AA);
}
//==============================================================================
float                                      //scaling factor
calc_scale(const Mat &X,                   //scaling basis vector
       const float width)              //width of desired shape
{
  int n = X.rows/2; float xmin = X.at<float>(0),xmax = X.at<float>(0);
  for(int i = 0; i < n; i++){
    xmin = min(xmin,X.at<float>(2*i));
    xmax = max(xmax,X.at<float>(2*i));
  }return width/(xmax-xmin);
}
//==============================================================================
int main(int argc,char** argv)
{
  //load data
  if(argc < 2){
    cout << "usage: ./visualise_shape_model shape_model" << endl; 
    return 0;
  }
  shape_model smodel = load_ft<shape_model>(argv[1]);
  
  //compute rigid parameters
  int n = smodel.V.rows/2;
  float scale = calc_scale(smodel.V.col(0),200);
  float tranx = n*150.0/smodel.V.col(2).dot(Mat::ones(2*n,1,CV_32F));
  float trany = n*150.0/smodel.V.col(3).dot(Mat::ones(2*n,1,CV_32F));

  //generate trajectory of parameters
  vector<float> val;
  for(int i = 0; i < 50; i++)val.push_back(float(i)/50);
  for(int i = 0; i < 50; i++)val.push_back(float(50-i)/50);
  for(int i = 0; i < 50; i++)val.push_back(-float(i)/50);
  for(int i = 0; i < 50; i++)val.push_back(-float(50-i)/50);

  //visualise
  Mat img(300,300,CV_8UC3); namedWindow("shape model");
  while(1){
    for(int k = 4; k < smodel.V.cols; k++){
      for(int j = 0; j < int(val.size()); j++){
    Mat p = Mat::zeros(smodel.V.cols,1,CV_32F);
    p.at<float>(0) = scale; p.at<float>(2) = tranx; p.at<float>(3) = trany;
    p.at<float>(k) = scale*val[j]*3.0*sqrt(smodel.e.at<float>(k)); 
    p.copyTo(smodel.p); img = Scalar::all(255); 
    char str[256]; sprintf(str,"mode: %d, val: %f sd",k-3,val[j]/3.0);
    draw_string(img,str);
    vector<Point2f> q = smodel.calc_shape(); 
    draw_shape(img,q,smodel.C);
    imshow("shape model",img);
    if(waitKey(10) == 'q')return 0;
      }
    }
  }return 0;
}
//==============================================================================
