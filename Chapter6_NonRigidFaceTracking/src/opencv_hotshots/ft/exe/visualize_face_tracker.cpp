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
  visualize_face_tracker: perform face tracking from a video/camera stream
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define fl at<float>
const char* usage = "usage: ./visualise_face_tracker tracker [video_file]";
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
bool
parse_help(int argc,char** argv)
{
  for(int i = 1; i < argc; i++){
    string str = argv[i];
    if(str.length() == 2){if(strcmp(str.c_str(),"-h") == 0)return true;}
    if(str.length() == 6){if(strcmp(str.c_str(),"--help") == 0)return true;}
  }return false;
}
//==============================================================================
int main(int argc,char** argv)
{
  //parse command line arguments
  if(parse_help(argc,argv)){cout << usage << endl; return 0;}
  if(argc < 2){cout << usage << endl; return 0;}
  
  //load detector model
  face_tracker tracker = load_ft<face_tracker>(argv[1]);

  //create tracker parameters
  face_tracker_params p; p.robust = false;
  p.ssize.resize(3);
  p.ssize[0] = Size(21,21);
  p.ssize[1] = Size(11,11);
  p.ssize[2] = Size(5,5);

  //open video stream
  VideoCapture cam; 
  if(argc > 2)cam.open(argv[2]); else cam.open(0);
  if(!cam.isOpened()){
    cout << "Failed opening video file." << endl
     << usage << endl; return 0;
  }
  //detect until user quits
  namedWindow("face tracker");
  while(cam.get(CV_CAP_PROP_POS_AVI_RATIO) < 0.999999){
    Mat im; cam >> im; 
    if(tracker.track(im,p))tracker.draw(im);
    draw_string(im,"d - redetection");
    tracker.timer.display_fps(im,Point(1,im.rows-1));
    imshow("face tracker",im);
    int c = waitKey(10);
    if(c == 'q')break;
    else if(c == 'd')tracker.reset();
  }
  destroyWindow("face tracker"); cam.release(); return 0;
}
//==============================================================================
