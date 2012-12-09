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
  visualize_patch_model: display the learned patch models to screen
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"      // For 'sprintf()'
#define fl at<float>
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
int
calc_height(const Mat &X,
        const float scale)
{
  int n = X.rows/2; 
  float ymin = scale*X.at<float>(1),ymax = scale*X.at<float>(1);
  for(int i = 0; i < n; i++){
    ymin = min(ymin,scale*X.at<float>(2*i+1));
    ymax = max(ymax,scale*X.at<float>(2*i+1));
  }return int(ymax-ymin+0.5);
}
//==============================================================================
int
parse_width(int argc,char** argv)
{
  for(int i = 1; i < argc; i++){
    string str = argv[i];
    if(str.length() != 2)continue;
    if(strcmp(str.c_str(),"-w") == 0){
      if(argc > i+1)return atoi(argv[i+1]);
    }
  }return 200;
}
//==============================================================================
int main(int argc,char** argv)
{
  //load data
  if(argc < 2){
    cout << "usage: ./visualise_patch_model patch_model [-w face_width]" 
     << endl; 
    return 0;
  }
  int width = parse_width(argc,argv);
  patch_models pmodel = load_ft<patch_models>(argv[1]);
  
  //compute scale factor
  float scale = calc_scale(pmodel.reference,width);
  int height = calc_height(pmodel.reference,scale);

  //compute image width
  int max_width = 0,max_height = 0;
  for(int i = 0; i < pmodel.n_patches(); i++){
    Size size = pmodel.patches[i].patch_size();
    max_width = max(max_width,int(scale*size.width));
    max_height = max(max_width,int(scale*size.height));
  }
  //create reference image
  Size image_size(width+4*max_width,height+4*max_height);
  Mat image(image_size.height,image_size.width,CV_8UC3); 
  image = Scalar::all(255);
  vector<Point> points(pmodel.n_patches());
  vector<Mat> P(pmodel.n_patches());
  for(int i = 0; i < pmodel.n_patches(); i++){
    Mat I1; normalize(pmodel.patches[i].P,I1,0,255,NORM_MINMAX);
    Mat I2; resize(I1,I2,Size(scale*I1.cols,scale*I1.rows));    
    Mat I3; I2.convertTo(I3,CV_8U); cvtColor(I3,P[i],CV_GRAY2RGB);
    points[i] = Point(scale*pmodel.reference.fl(2*i  ) + 
              image_size.width /2 - P[i].cols/2,
              scale*pmodel.reference.fl(2*i+1) + 
              image_size.height/2 - P[i].rows/2);
    Mat I = image(Rect(points[i].x,points[i].y,P[i].cols,P[i].rows));
    P[i].copyTo(I);
  }
  //animate
  namedWindow("patch model");
  int i = 0;
  while(1){
    Mat img = image.clone();
    Mat I = img(Rect(points[i].x,points[i].y,P[i].cols,P[i].rows));
    P[i].copyTo(I);
    rectangle(img,points[i],Point(points[i].x+P[i].cols,points[i].y+P[i].rows),
          CV_RGB(255,0,0),2,CV_AA);
    char str[256]; sprintf(str,"patch %d",i); draw_string(img,str);
    imshow("patch model",img);
    int c = waitKey(0);
    if(c == 'q')break;
    else if(c == 'p')i++;
    else if(c == 'o')i--;
    if(i < 0)i = 0; else if(i >= pmodel.n_patches())i = pmodel.n_patches()-1;
  }
  destroyWindow("patch model"); return 0;
}
//==============================================================================
