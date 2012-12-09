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
  face_detector: Shape initializer for face tracking
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/face_detector.hpp"
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define fl at<float>
//==============================================================================
vector<Point2f>
face_detector::
detect(const Mat &im,
       const float scaleFactor,
       const int minNeighbours,
       const Size minSize)
{
  //convert image to greyscale
  Mat gray; if(im.channels()==1)gray = im; else cvtColor(im,gray,CV_RGB2GRAY);

  //detect faces
  vector<Rect> faces; Mat eqIm; equalizeHist(gray,eqIm);
  detector.detectMultiScale(eqIm,faces,scaleFactor,minNeighbours,0
                |CV_HAAR_FIND_BIGGEST_OBJECT
                |CV_HAAR_SCALE_IMAGE,minSize);
  if(faces.size() < 1){return vector<Point2f>();}
  
  //predict face placement
  Rect R = faces[0]; Vec3f scale = detector_offset*R.width;
  int n = reference.rows/2; vector<Point2f> p(n);
  for(int i = 0; i < n; i++){
    p[i].x = scale[2]*reference.fl(2*i  ) + R.x + 0.5 * R.width  + scale[0];
    p[i].y = scale[2]*reference.fl(2*i+1) + R.y + 0.5 * R.height + scale[1];
  }return p;
}
//==============================================================================
void
face_detector::
train(ft_data &data,
      const string fname,
      const Mat &ref,
      const bool mirror,
      const bool visi,
      const float frac,
      const float scaleFactor,
      const int minNeighbours,
      const Size minSize)
{
  detector.load(fname.c_str()); detector_fname = fname; reference = ref.clone();
  vector<float> xoffset(0),yoffset(0),zoffset(0);
  for(int i = 0; i < data.n_images(); i++){
    Mat im = data.get_image(i,0); if(im.empty())continue;
    vector<Point2f> p = data.get_points(i,false); int n = p.size();
    Mat pt = Mat(p).reshape(1,2*n);
    vector<Rect> faces; Mat eqIm; equalizeHist(im,eqIm);
    detector.detectMultiScale(eqIm,faces,scaleFactor,minNeighbours,0
                  |CV_HAAR_FIND_BIGGEST_OBJECT
                  |CV_HAAR_SCALE_IMAGE,minSize);
    if(faces.size() >= 1){
      if(visi){
    Mat I; cvtColor(im,I,CV_GRAY2RGB);
    for(int i = 0; i < n; i++)circle(I,p[i],1,CV_RGB(0,255,0),2,CV_AA);
    rectangle(I,faces[0].tl(),faces[0].br(),CV_RGB(255,0,0),3);
    imshow("face detector training",I); waitKey(10); 
      }
      //check if enough points are in detected rectangle
      if(this->enough_bounded_points(pt,faces[0],frac)){
    Point2f center = this->center_of_mass(pt); float w = faces[0].width;
    xoffset.push_back((center.x - (faces[0].x+0.5*faces[0].width ))/w);
    yoffset.push_back((center.y - (faces[0].y+0.5*faces[0].height))/w);
    zoffset.push_back(this->calc_scale(pt)/w);
      }
    }
    if(mirror){
      im = data.get_image(i,1); if(im.empty())continue;
      p = data.get_points(i,true);
      pt = Mat(p).reshape(1,2*n);
      equalizeHist(im,eqIm);
      detector.detectMultiScale(eqIm,faces,scaleFactor,minNeighbours,0
                  |CV_HAAR_FIND_BIGGEST_OBJECT
                |CV_HAAR_SCALE_IMAGE,minSize);
      if(faces.size() >= 1){
    if(visi){
      Mat I; cvtColor(im,I,CV_GRAY2RGB);
      for(int i = 0; i < n; i++)circle(I,p[i],1,CV_RGB(0,255,0),2,CV_AA);
      rectangle(I,faces[0].tl(),faces[0].br(),CV_RGB(255,0,0),3);
      imshow("face detector training",I); waitKey(10);
    }
    //check if enough points are in detected rectangle
    if(this->enough_bounded_points(pt,faces[0],frac)){
      Point2f center = this->center_of_mass(pt); float w = faces[0].width;
      xoffset.push_back((center.x - (faces[0].x+0.5*faces[0].width ))/w);
      yoffset.push_back((center.y - (faces[0].y+0.5*faces[0].height))/w);
      zoffset.push_back(this->calc_scale(pt)/w);
    }
      }
    }
  }
  //choose median value
  Mat X = Mat(xoffset),Xsort,Y = Mat(yoffset),Ysort,Z = Mat(zoffset),Zsort;
  cv::sort(X,Xsort,CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING); int nx = Xsort.rows;
  cv::sort(Y,Ysort,CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING); int ny = Ysort.rows;
  cv::sort(Z,Zsort,CV_SORT_EVERY_COLUMN|CV_SORT_ASCENDING); int nz = Zsort.rows;
  detector_offset = Vec3f(Xsort.fl(nx/2),Ysort.fl(ny/2),Zsort.fl(nz/2)); return;
}
//==============================================================================
bool
face_detector::
enough_bounded_points(const Mat &pts,
              const Rect R,
              const float frac)
{
  int n = pts.rows/2,m = 0;
  for(int i = 0; i < n; i++){
    if((pts.fl(2*i  ) >= R.x) && (pts.fl(2*i  ) <= R.x + R.width) &&
       (pts.fl(2*i+1) >= R.y) && (pts.fl(2*i+1) <= R.y + R.height))m++;
  }
  if(float(m)/n >= frac)return true; else return false;
}
//==============================================================================
Point2f
face_detector::
center_of_mass(const Mat &pts)
{
  float mx = 0,my = 0; int n = pts.rows/2;
  for(int i = 0; i < n; i++){
    mx += pts.fl(2*i); my += pts.fl(2*i+1);
  }return Point2f(mx/n,my/n);
}
//==============================================================================
float 
face_detector::
calc_scale(const Mat &pts)
{
  Point2f c = this->center_of_mass(pts); int n = pts.rows/2;
  Mat p(2*n,1,CV_32F);
  for(int i = 0; i < n; i++){
    p.fl(2*i  ) = pts.fl(2*i  ) - c.x;
    p.fl(2*i+1) = pts.fl(2*i+1) - c.y;
  }return reference.dot(p)/reference.dot(reference);
}
//==============================================================================
void 
face_detector::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); 
  fs << "{"
     << "fname"     << detector_fname
     << "x offset"  << detector_offset[0]
     << "y offset"  << detector_offset[1]
     << "z offset"  << detector_offset[2]
     << "reference" << reference
     << "}";
}
//==============================================================================
void 
face_detector::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP);
  node["fname"]     >> detector_fname;
  node["x offset"]  >> detector_offset[0];
  node["y offset"]  >> detector_offset[1];
  node["z offset"]  >> detector_offset[2];
  node["reference"] >> reference;
  detector.load(detector_fname.c_str());
}
//==============================================================================
