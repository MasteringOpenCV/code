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
  patch_model: Correlation patch experts
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/patch_model.hpp"
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "stdio.h"      // For 'sprintf()'

#define fl at<float>
//==============================================================================
Mat
patch_model::
convert_image(const Mat &im)
{
  Mat I; 
  if(im.channels() == 1){
    if(im.type() != CV_32F)im.convertTo(I,CV_32F); 
    else I = im;
  }else{
    if(im.channels() == 3){
      Mat img; cvtColor(im,img,CV_RGB2GRAY);
      if(img.type() != CV_32F)img.convertTo(I,CV_32F); 
      else I = img;
    }else{cout << "Unsupported image type!" << endl; abort();}
  }
  I += 1.0; log(I,I); return I;
}
//==============================================================================
Mat 
patch_model::
calc_response(const Mat &im,const bool sum2one)
{
  Mat I = this->convert_image(im);
  Mat res; matchTemplate(I,P,res,CV_TM_CCOEFF_NORMED); 
  if(sum2one){
    normalize(res,res,0,1,NORM_MINMAX); res /= sum(res)[0];
  }return res;
}
//==============================================================================
void 
patch_model::
train(const vector<Mat> &images,
      const Size psize,
      const float var,
      const float lambda,
      const float mu_init,
      const int nsamples,
      const bool visi)
{
  int N = images.size(),n = psize.width*psize.height;

  //compute desired response map
  Size wsize = images[0].size();
  if((wsize.width < psize.width) || (wsize.height < psize.height)){
    cerr << "Invalid image size < patch size!" << endl; throw std::exception();
  }
  int dx = wsize.width-psize.width,dy = wsize.height-psize.height;
  Mat F(dy,dx,CV_32F);
  for(int y = 0; y < dy; y++){   float vy = (dy-1)/2 - y;
    for(int x = 0; x < dx; x++){ float vx = (dx-1)/2 - x;
      F.fl(y,x) = exp(-0.5*(vx*vx+vy*vy)/var);
    }
  }
  normalize(F,F,0,1,NORM_MINMAX);

  //allocate memory
  Mat I(wsize.height,wsize.width,CV_32F);
  Mat dP(psize.height,psize.width,CV_32F);
  Mat O = Mat::ones(psize.height,psize.width,CV_32F)/n;
  P = Mat::zeros(psize.height,psize.width,CV_32F);

  //optimise using stochastic gradient descent
  RNG rn(getTickCount()); double mu=mu_init,step=pow(1e-8/mu_init,1.0/nsamples);
  for(int sample = 0; sample < nsamples; sample++){ int i = rn.uniform(0,N);
    I = this->convert_image(images[i]); dP = 0.0;
    for(int y = 0; y < dy; y++){
      for(int x = 0; x < dx; x++){
    Mat Wi = I(Rect(x,y,psize.width,psize.height)).clone();
    Wi -= Wi.dot(O); normalize(Wi,Wi);
    dP += (F.fl(y,x) - P.dot(Wi))*Wi;
      }
    }    
    P += mu*(dP - lambda*P); mu *= step;
    if(visi){
      Mat R; matchTemplate(I,P,R,CV_TM_CCOEFF_NORMED);
      Mat PP; normalize(P,PP,0,1,NORM_MINMAX);
      normalize(dP,dP,0,1,NORM_MINMAX);
      normalize(R,R,0,1,NORM_MINMAX);
      imshow("P",PP); imshow("dP",dP); imshow("R",R); 
      if(waitKey(10) == 27)break;
    }
  }return;
}
//==============================================================================
void
patch_model::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); fs << "{" << "P"  << P << "}";
}  
//==============================================================================
void
patch_model::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP); node["P"] >> P; 
}
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
void 
patch_models::
train(ft_data &data,
      const vector<Point2f> &ref,
      const Size psize,
      const Size ssize,
      const bool mirror,
      const float var,
      const float lambda,
      const float mu_init,
      const int nsamples,
      const bool visi)
{
  //set reference shape
  int n = ref.size(); reference = Mat(ref).reshape(1,2*n);
  Size wsize = psize + ssize;

  //train each patch model in turn
  patches.resize(n);
  for(int i = 0; i < n; i++){
    if(visi)cout << "training patch " << i << "..." << endl;
    vector<Mat> images(0);
    for(int j = 0; j < data.n_images(); j++){
      Mat im = data.get_image(j,0); 
      vector<Point2f> p = data.get_points(j,false);
      Mat pt = Mat(p).reshape(1,2*n);
      Mat S = this->calc_simil(pt),A(2,3,CV_32F); 
      A.fl(0,0) = S.fl(0,0); A.fl(0,1) = S.fl(0,1);
      A.fl(1,0) = S.fl(1,0); A.fl(1,1) = S.fl(1,1);
      A.fl(0,2) = pt.fl(2*i  ) - 
    (A.fl(0,0) * (wsize.width-1)/2 + A.fl(0,1)*(wsize.height-1)/2);
      A.fl(1,2) = pt.fl(2*i+1) - 
    (A.fl(1,0) * (wsize.width-1)/2 + A.fl(1,1)*(wsize.height-1)/2);
      Mat I; warpAffine(im,I,A,wsize,INTER_LINEAR+WARP_INVERSE_MAP);
      images.push_back(I);
      if(mirror){
    im = data.get_image(j,1); 
    p = data.get_points(j,true);
    pt = Mat(p).reshape(1,2*n);
    S = this->calc_simil(pt);
    A.fl(0,0) = S.fl(0,0); A.fl(0,1) = S.fl(0,1);
    A.fl(1,0) = S.fl(1,0); A.fl(1,1) = S.fl(1,1);
    A.fl(0,2) = pt.fl(2*i  ) - 
      (A.fl(0,0) * (wsize.width-1)/2 + A.fl(0,1)*(wsize.height-1)/2);
    A.fl(1,2) = pt.fl(2*i+1) - 
      (A.fl(1,0) * (wsize.width-1)/2 + A.fl(1,1)*(wsize.height-1)/2);
    warpAffine(im,I,A,wsize,INTER_LINEAR+WARP_INVERSE_MAP);
    images.push_back(I);
      }
    }
    patches[i].train(images,psize,var,lambda,mu_init,nsamples,visi);
  }
}
//==============================================================================
vector<Point2f> 
patch_models::
calc_peaks(const Mat &im,
       const vector<Point2f> &points,
       const Size ssize)
{
  int n = points.size(); assert(n == int(patches.size()));
  Mat pt = Mat(points).reshape(1,2*n);
  Mat S = this->calc_simil(pt);
  Mat Si = this->inv_simil(S);
  vector<Point2f> pts = this->apply_simil(Si,points);
  for(int i = 0; i < n; i++){
    Size wsize = ssize + patches[i].patch_size(); Mat A(2,3,CV_32F);     
    A.fl(0,0) = S.fl(0,0); A.fl(0,1) = S.fl(0,1);
    A.fl(1,0) = S.fl(1,0); A.fl(1,1) = S.fl(1,1);
    A.fl(0,2) = pt.fl(2*i  ) - 
      (A.fl(0,0) * (wsize.width-1)/2 + A.fl(0,1)*(wsize.height-1)/2);
    A.fl(1,2) = pt.fl(2*i+1) - 
      (A.fl(1,0) * (wsize.width-1)/2 + A.fl(1,1)*(wsize.height-1)/2);
    Mat I; warpAffine(im,I,A,wsize,INTER_LINEAR+WARP_INVERSE_MAP);
    Mat R = patches[i].calc_response(I,false);
    Point maxLoc; minMaxLoc(R,0,0,0,&maxLoc);
    pts[i] = Point2f(pts[i].x + maxLoc.x - 0.5*ssize.width,
             pts[i].y + maxLoc.y - 0.5*ssize.height);
  }return this->apply_simil(S,pts);
}
//=============================================================================
vector<Point2f> 
patch_models::
apply_simil(const Mat &S,
        const vector<Point2f> &points)
{
  int n = points.size();
  vector<Point2f> p(n);
  for(int i = 0; i < n; i++){
    p[i].x = S.fl(0,0)*points[i].x + S.fl(0,1)*points[i].y + S.fl(0,2);
    p[i].y = S.fl(1,0)*points[i].x + S.fl(1,1)*points[i].y + S.fl(1,2);
  }return p;
}
//=============================================================================
Mat 
patch_models::
inv_simil(const Mat &S)
{
  Mat Si(2,3,CV_32F);
  float d = S.fl(0,0)*S.fl(1,1) - S.fl(1,0)*S.fl(0,1);
  Si.fl(0,0) = S.fl(1,1)/d; Si.fl(0,1) = -S.fl(0,1)/d;
  Si.fl(1,1) = S.fl(0,0)/d; Si.fl(1,0) = -S.fl(1,0)/d;
  Mat Ri = Si(Rect(0,0,2,2));
  Mat t = -Ri*S.col(2),St = Si.col(2); t.copyTo(St); return Si; 
}
//=============================================================================
Mat
patch_models::
calc_simil(const Mat &pts)
{
  //compute translation
  int n = pts.rows/2; float mx = 0,my = 0;
  for(int i = 0; i < n; i++){
    mx += pts.fl(2*i); my += pts.fl(2*i+1);
  }  
  Mat p(2*n,1,CV_32F); mx /= n; my /= n;
  for(int i = 0; i < n; i++){
    p.fl(2*i) = pts.fl(2*i) - mx; p.fl(2*i+1) = pts.fl(2*i+1) - my;
  }
  //compute rotation and scale
  float a=0,b=0,c=0;
  for(int i = 0; i < n; i++){
    a += reference.fl(2*i) * reference.fl(2*i  ) + 
      reference.fl(2*i+1) * reference.fl(2*i+1);
    b += reference.fl(2*i) * p.fl(2*i  ) + reference.fl(2*i+1) * p.fl(2*i+1);
    c += reference.fl(2*i) * p.fl(2*i+1) - reference.fl(2*i+1) * p.fl(2*i  );
  }
  b /= a; c /= a;
  float scale = sqrt(b*b+c*c),theta = atan2(c,b); 
  float sc = scale*cos(theta),ss = scale*sin(theta);
  return (Mat_<float>(2,3) << sc,-ss,mx,ss,sc,my);
}
//==============================================================================
void 
patch_models::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); 
  fs << "{" << "reference" << reference;
  fs << "n_patches" << (int)patches.size();
  for(int i = 0; i < int(patches.size()); i++){
    char str[256]; const char* ss;
    sprintf(str,"patch %d",i); ss = str; fs << ss << patches[i];
  }
  fs << "}";
}
//==============================================================================
void 
patch_models::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP); 
  node["reference"] >> reference;
  int n; node["n_patches"] >> n; patches.resize(n);
  for(int i = 0; i < n; i++){
    char str[256]; const char* ss;
    sprintf(str,"patch %d",i); ss = str; node[ss] >> patches[i];
  }
}
//==============================================================================
