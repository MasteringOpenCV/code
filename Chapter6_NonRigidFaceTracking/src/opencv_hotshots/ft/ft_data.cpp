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
  ft_data: face tracker data 
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft_data.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"	// For 'sprintf()'

//==============================================================================
void
ft_data::
rm_incomplete_samples()
{
  int n = points[0].size(),N = points.size();
  for(int i = 1; i < N; i++)n = max(n,int(points[i].size()));
  for(int i = 0; i < int(points.size()); i++){
    if(int(points[i].size()) != n){
      points.erase(points.begin()+i); imnames.erase(imnames.begin()+i); i--;
    }else{
      int j = 0;
      for(; j < n; j++){
    if((points[i][j].x <= 0) || (points[i][j].y <= 0))break;
      }
      if(j < n){
    points.erase(points.begin()+i); imnames.erase(imnames.begin()+i); i--;
      }
    }
  }
}
//==============================================================================
void
ft_data::
rm_sample(const int idx)
{
  if((idx < 0) || (idx >= int(imnames.size())))return;
  points.erase(points.begin()+idx); imnames.erase(imnames.begin()+idx);
}
//==============================================================================
Mat
ft_data::
get_image(const int idx,
      const int flag)
{
  if((idx < 0) || (idx >= (int)imnames.size()))return Mat();
  Mat img,im;
  if(flag < 2)img = imread(imnames[idx],0); else img = imread(imnames[idx],1);
  if(flag % 2 != 0)flip(img,im,1); else im = img;
  return im;
}
//==============================================================================
vector<Point2f>
ft_data::
get_points(const int idx,
       const bool flipped)
{
  if((idx < 0) || (idx >= (int)imnames.size()))return vector<Point2f>();
  vector<Point2f> p = points[idx];
  if(flipped){
    Mat im = this->get_image(idx,0); int n = p.size(); vector<Point2f> q(n);
    for(int i = 0; i < n; i++){      
      q[i].x = im.cols-1-p[symmetry[i]].x;
      q[i].y = p[symmetry[i]].y;
    }return q;
  }else return p;
}
//==============================================================================
void
ft_data::
draw_points(Mat &im,
        const int idx,
        const bool flipped,
        const Scalar color,
        const vector<int> &pts)
{
  if((idx < 0) || (idx >= (int)imnames.size()))return;
  int n = points[idx].size();
  if(pts.size() == 0){
    for(int i = 0; i < n; i++){
      if(!flipped)circle(im,points[idx][i],1,color,2,CV_AA);
      else{
    Point2f p(im.cols - 1 - points[idx][symmetry[i]].x,
          points[idx][symmetry[i]].y);
    circle(im,p,1,color,2,CV_AA);
      }
    }
  }else{
    int m = pts.size();
    for(int j = 0; j < m; j++){
      int i = pts[j]; if((i < 0) || (i >= n))continue;
      if(!flipped)circle(im,points[idx][i],1,color,2,CV_AA);
      else{
    Point2f p(im.cols - 1 - points[idx][symmetry[i]].x,
          points[idx][symmetry[i]].y);
    circle(im,p,1,color,2,CV_AA);
      }
    }
  }
}
//==============================================================================
void
ft_data::
draw_sym(Mat &im,
     const int idx,
     const bool flipped,
     const vector<int> &pts)
{
  if((idx < 0) || (idx >= (int)imnames.size()))return;
  int n = points[idx].size();
  RNG rn; vector<Scalar> colors(n); 
  for(int i = 0; i < n; i++)colors[i] = Scalar::all(0.0);
  for(int i = 0; i < n; i++){
    if(colors[i] == Scalar::all(0.0)){
      colors[i] = Scalar(rn.uniform(0,255),rn.uniform(0,255),rn.uniform(0,255));
      colors[symmetry[i]] = colors[i];
    }
  }
  vector<Point2f> p = this->get_points(idx,flipped); 
  if(pts.size() == 0){
    for(int i = 0; i < n; i++){circle(im,p[i],1,colors[i],2,CV_AA);}
  }else{
    int m = pts.size();
    for(int j = 0; j < m; j++){
      int i = pts[j]; if((i < 0) || (i >= n))continue;
      circle(im,p[i],1,colors[i],2,CV_AA);
    }
  }
}
//==============================================================================
void
ft_data::
draw_connect(Mat &im,
         const int idx,
         const bool flipped,
         const Scalar color,
         const vector<int> &con)
{
  if((idx < 0) || (idx >= (int)imnames.size()))return;
  int n = connections.size();
  if(con.size() == 0){    
    for(int i = 0; i < n; i++){
      int j = connections[i][0],k = connections[i][1];
      if(!flipped)line(im,points[idx][j],points[idx][k],color,1);
      else{
    Point2f p(im.cols - 1 - points[idx][symmetry[j]].x,
          points[idx][symmetry[j]].y);
    Point2f q(im.cols - 1 - points[idx][symmetry[k]].x,
          points[idx][symmetry[k]].y);
    line(im,p,q,color,1);
      }
    }
  }else{
    int m = con.size();
    for(int j = 0; j < m; j++){
      int i = con[j]; if((i < 0) || (i >= n))continue;
      int k = connections[i][0],l = connections[i][1];
      if(!flipped)line(im,points[idx][k],points[idx][l],color,1);
      else{
    Point2f p(im.cols - 1 - points[idx][symmetry[k]].x,
          points[idx][symmetry[k]].y);
    Point2f q(im.cols - 1 - points[idx][symmetry[l]].x,
          points[idx][symmetry[l]].y);
    line(im,p,q,color,1);
      }
    }
  }
}
//============================================================================= 
void 
ft_data::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); 
  fs << "{";
  fs << "n_connections" << (int)connections.size();
  for(int i = 0; i < int(connections.size()); i++){
    char str[256]; const char* ss;
    sprintf(str,"connections %d 0",i); ss = str; fs << ss << connections[i][0];
    sprintf(str,"connections %d 1",i); ss = str; fs << ss << connections[i][1];
  }
  fs << "n_symmetry" << (int)symmetry.size();
  for(int i = 0; i < int(symmetry.size()); i++){
    char str[256]; const char* ss;
    sprintf(str,"symmetry %d",i); ss = str; fs << ss << symmetry[i];
  }
  fs << "n_images" << (int)imnames.size();
  for(int i = 0; i < int(imnames.size()); i++){
    char str[256]; const char* ss;
    sprintf(str,"image %d",i); ss = str; fs << ss << imnames[i];
  }
  int n = points[0].size(),N = points.size();
  Mat X(2*n,N,CV_32F); X = -1;
  for(int i = 0; i < N; i++){
    if(int(points[i].size()) == n){
      for(int j = 0; j < n; j++){
    X.at<float>(2*j  ,i) = points[i][j].x;
    X.at<float>(2*j+1,i) = points[i][j].y;
      }
    }
  }
  fs << "shapes" << X << "}";
}
//==============================================================================
void
ft_data::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP);
  int n; node["n_connections"] >> n; connections.resize(n);
  for(int i = 0; i < n; i++){
    char str[256]; const char* ss;
    sprintf(str,"connections %d 0",i); ss = str; node[ss] >> connections[i][0];
    sprintf(str,"connections %d 1",i); ss = str; node[ss] >> connections[i][1];
  }
  node["n_symmetry"] >> n; symmetry.resize(n);
  for(int i = 0; i < n; i++){
    char str[256]; const char* ss;
    sprintf(str,"symmetry %d",i); ss = str; node[ss] >> symmetry[i];
  }
  node["n_images"] >> n; imnames.resize(n);
  for(int i = 0; i < n; i++){
    char str[256]; const char* ss;
    sprintf(str,"image %d",i); ss = str; node[ss] >> imnames[i];
  }
  Mat X; node["shapes"] >> X; int N = X.cols; n = X.rows/2; 
  points.resize(N);
  for(int i = 0; i < N; i++){
    points[i].clear();
    for(int j = 0; j < n; j++){
      Point2f p(X.at<float>(2*j,i),X.at<float>(2*j+1,i));
      if((p.x >= 0) && (p.y >= 0))points[i].push_back(p);
    }
  }
}
//==============================================================================

