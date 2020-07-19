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
  face_tracker: face tracking classes
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/face_tracker.hpp"
#include "opencv_hotshots/ft/ft.hpp"
#include <iostream>
#include "stdio.h"      // For 'sprintf()'
#define fl at<float>
#include "opencv_hotshots/ft/fps_timer.hpp"

//==============================================================================
//==============================================================================
//==============================================================================
//========================== face_tracker_params ===============================
//==============================================================================
//==============================================================================
//==============================================================================
face_tracker_params::
face_tracker_params()
{
  ssize.resize(3); 
  ssize[0] = Size(21,21); ssize[1] = Size(11,11); ssize[2] = Size(5,5);
  robust = false; itol = 20; ftol = 1e-3;
  scaleFactor = 1.1; minNeighbours = 2; minSize = Size(30,30);
}
//==============================================================================
void 
face_tracker_params::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); fs << "{";
  fs << "nlevels" << int(ssize.size());
  for(int i = 0; i < int(ssize.size()); i++){ char str[256]; const char* ss;
    sprintf(str,"w %d",i); ss = str; fs << ss << ssize[i].width;
    sprintf(str,"h %d",i); ss = str; fs << ss << ssize[i].height;
  }
  fs << "robust" << robust
     << "itol" << itol
     << "ftol" << ftol
     << "scaleFactor" << scaleFactor
     << "minNeighbours" << minNeighbours
     << "minWidth" << minSize.width
     << "minHeight" << minSize.height
     << "}";
}
//==============================================================================
void 
face_tracker_params::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP);
  int n; node["nlevels"] >> n; ssize.resize(n);
  for(int i = 0; i < n; i++){ char str[256]; const char* ss;
    sprintf(str,"w %d",i); ss = str; node[ss] >> ssize[i].width;
    sprintf(str,"h %d",i); ss = str; node[ss] >> ssize[i].height;
  }
  node["robust"] >> robust;
  node["itol"] >> itol;
  node["ftol"] >> ftol;
  node["scaleFactor"] >> scaleFactor;
  node["minNeighbours"] >> minNeighbours;
  node["minWidth"] >> minSize.width;
  node["minHeight"] >> minSize.height;
}
//==============================================================================
void 
write(FileStorage& fs, 
      const string&, 
      const face_tracker_params& x)
{
  x.write(fs);
}
//==============================================================================
void 
read(const FileNode& node, 
     face_tracker_params& x,
     const face_tracker_params& d)
{
  if(node.empty())x = d; else x.read(node);
}
//==============================================================================
face_tracker_params 
load_face_tracker_params(const char* fname)
{
  face_tracker_params x; FileStorage f(fname,FileStorage::READ);
  f["face_tracker_params"] >> x; f.release(); return x;
}
//==============================================================================
void 
save_face_tracker_params(const char* fname,
             const face_tracker_params& x)
{
  FileStorage f(fname,FileStorage::WRITE);
  f << "face_tracker_params" << x; f.release();
}
//==============================================================================
//==============================================================================
//==============================================================================
//============================== face_tracker ==================================
//==============================================================================
//==============================================================================
//==============================================================================
int
face_tracker::
track(const Mat &im,const face_tracker_params &p)
{
  //convert image to greyscale
  Mat gray; if(im.channels()==1)gray = im; else cvtColor(im,gray,CV_RGB2GRAY);

  //initialise
  if(!tracking)
    points = detector.detect(gray,p.scaleFactor,p.minNeighbours,p.minSize);
  if((int)points.size() != smodel.npts())return 0;

  //fit
  for(int level = 0; level < int(p.ssize.size()); level++)
    points = this->fit(gray,points,p.ssize[level],p.robust,p.itol,p.ftol);

  //set tracking flag and increment timer
  tracking = true; timer.increment();  return 1;
}
//==============================================================================
void
face_tracker::
draw(Mat &im,
     const Scalar pts_color,
     const Scalar con_color)
{
  int n = points.size(); if(n == 0)return;
  for(int i = 0; i < smodel.C.rows; i++){
    int j = smodel.C.at<int>(i,0),k = smodel.C.at<int>(i,1);
    line(im,points[j],points[k],con_color,1);
  }
  for(int i = 0; i < n; i++)circle(im,points[i],1,pts_color,2,CV_AA);
}
//==============================================================================
vector<Point2f>
face_tracker::
fit(const Mat &image,
    const vector<Point2f> &init,
    const Size ssize,
    const bool robust,
    const int itol,
    const float ftol)
{
  int n = smodel.npts(); 
  assert((int(init.size())==n) && (pmodel.n_patches()==n));
  smodel.calc_params(init); vector<Point2f> pts = smodel.calc_shape();

  //find facial features in image around current estimates
  vector<Point2f> peaks = pmodel.calc_peaks(image,pts,ssize);

  //optimise
  if(!robust){
    smodel.calc_params(peaks); //compute shape model parameters        
    pts = smodel.calc_shape(); //update shape
  }else{
    Mat weight(n,1,CV_32F),weight_sort(n,1,CV_32F);
    vector<Point2f> pts_old = pts;
    for(int iter = 0; iter < itol; iter++){
      //compute robust weight
      for(int i = 0; i < n; i++)weight.fl(i) = norm(pts[i] - peaks[i]);
      cv::sort(weight,weight_sort,CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING);
      double var = 1.4826*weight_sort.fl(n/2); if(var < 0.1)var = 0.1;
      pow(weight,2,weight); weight *= -0.5/(var*var); cv::exp(weight,weight); 

      //compute shape model parameters    
      smodel.calc_params(peaks,weight);
      
      //update shape
      pts = smodel.calc_shape();
      
      //check for convergence
      float v = 0; for(int i = 0; i < n; i++)v += norm(pts[i]-pts_old[i]);
      if(v < ftol)break; else pts_old = pts;
    }
  }return pts;
}
//==============================================================================
void 
face_tracker::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); 
  fs << "{"
     << "detector" << detector
     << "smodel"   << smodel
     << "pmodel"   << pmodel
     << "}";
}
//==============================================================================
void 
face_tracker::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP);
  node["detector"] >> detector;
  node["smodel"]   >> smodel;
  node["pmodel"]   >> pmodel;
}
//==============================================================================
