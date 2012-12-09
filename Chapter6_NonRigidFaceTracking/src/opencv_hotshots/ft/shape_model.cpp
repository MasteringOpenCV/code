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
  shape_model: A combined local-global 2D point distribution model
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/shape_model.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#define fl at<float>
//==============================================================================
void 
shape_model::
calc_params(const vector<Point2f> &pts,const Mat weight,const float c_factor)
{
  int n = pts.size(); assert(V.rows == 2*n);
  Mat s = Mat(pts).reshape(1,2*n); //point set to vector format
  if(weight.empty())p = V.t()*s;   //simple projection
  else{                            //scaled projection
    if(weight.rows != n){cout << "Invalid weighting matrix" << endl; abort();}
    int K = V.cols; Mat H = Mat::zeros(K,K,CV_32F),g = Mat::zeros(K,1,CV_32F);
    for(int i = 0; i < n; i++){
      Mat v = V(Rect(0,2*i,K,2)); float w = weight.fl(i);
      H += w*v.t()*v; g += w*v.t()*Mat(pts[i]);
    }
    solve(H,g,p,DECOMP_SVD);
  }this->clamp(c_factor);          //clamp resulting parameters
}
//==============================================================================
Mat
shape_model::
center_shape(const Mat &pts)
{
  int n = pts.rows/2; float mx = 0,my = 0;
  for(int i = 0; i < n; i++){
    mx += pts.fl(2*i); my += pts.fl(2*i+1);
  }  
  Mat p(2*n,1,CV_32F); mx /= n; my /= n;
  for(int i = 0; i < n; i++){
    p.fl(2*i) = pts.fl(2*i) - mx; p.fl(2*i+1) = pts.fl(2*i+1) - my;
  }return p;
}
//==============================================================================
vector<Point2f> 
shape_model::
calc_shape()
{
  Mat s = V*p; int n = s.rows/2; vector<Point2f> pts;
  for(int i = 0; i < n; i++)pts.push_back(Point2f(s.fl(2*i),s.fl(2*i+1)));
  return pts;
}
//==============================================================================
void 
shape_model::
set_identity_params()
{
  p = 0.0; p.fl(0) = 1.0; //1'st parameter is scale
}
//==============================================================================
void
shape_model::
train(const vector<vector<Point2f> > &points,
      const vector<Vec2i> &con,
      const float frac,
      const int kmax)
{
  //vectorize points
  Mat X = this->pts2mat(points);
  int N = X.cols,n = X.rows/2;

  //align shapes
  Mat Y = this->procrustes(X);

  //compute rigid transformation
  Mat R = this->calc_rigid_basis(Y);

  //compute non-rigid transformation
  Mat P = R.t()*Y; Mat dY = Y - R*P; SVD svd(dY*dY.t());
  int m = min(min(kmax,N-1),n-1);
  float vsum = 0; for(int i = 0; i < m; i++)vsum += svd.w.fl(i);
  float v = 0; int k = 0; 
  for(k = 0; k < m; k++){v += svd.w.fl(k); if(v/vsum >= frac){k++; break;}}
  if(k > m)k = m;
  Mat D = svd.u(Rect(0,0,k,2*n));

  //combine bases
  V.create(2*n,4+k,CV_32F);
  Mat Vr = V(Rect(0,0,4,2*n)); R.copyTo(Vr);
  Mat Vd = V(Rect(4,0,k,2*n)); D.copyTo(Vd);

  //compute variance (normalized wrt scale)
  Mat Q = V.t()*X;
  for(int i = 0; i < N; i++){
    float v = Q.fl(0,i); Mat q = Q.col(i); q /= v; 
  }
  e.create(4+k,1,CV_32F);
  pow(Q,2,Q);
  for(int i = 0; i < 4+k; i++){
    if(i < 4)e.fl(i) = -1;
    else e.fl(i) = Q.row(i).dot(Mat::ones(1,N,CV_32F))/(N-1);
  }
  //store connectivity
  if(con.size() > 0){ //default connectivity
    int m = con.size();
    C.create(m,2,CV_32F);
    for(int i = 0; i < m; i++){
      C.at<int>(i,0) = con[i][0]; C.at<int>(i,1) = con[i][1];
    }
  }else{              //user-specified connectivity
    C.create(n,2,CV_32S);
    for(int i = 0; i < n-1; i++){
      C.at<int>(i,0) = i; C.at<int>(i,1) = i+1;
    }    
    C.at<int>(n-1,0) = n-1; C.at<int>(n-1,1) = 0;
  }
}
//==============================================================================
Mat
shape_model::
pts2mat(const vector<vector<Point2f> > &points)
{
  int N = points.size(); assert(N > 0);
  int n = points[0].size();
  for(int i = 1; i < N; i++)assert(int(points[i].size()) == n); 
  Mat X(2*n,N,CV_32F);
  for(int i = 0; i < N; i++){
    Mat x = X.col(i),y = Mat(points[i]).reshape(1,2*n); y.copyTo(x);
  }return X;
}
//==============================================================================
Mat 
shape_model::
procrustes(const Mat &X,
       const int itol,
       const float ftol)
{
  int N = X.cols,n = X.rows/2;

  //remove centre of mass
  Mat P = X.clone();
  for(int i = 0; i < N; i++){
    Mat p = P.col(i);
    float mx = 0,my = 0;
    for(int j = 0; j < n; j++){mx += p.fl(2*j); my += p.fl(2*j+1);}
    mx /= n; my /= n;
    for(int j = 0; j < n; j++){p.fl(2*j) -= mx; p.fl(2*j+1) -= my;}
  }
  //optimise scale and rotation
  Mat C_old;
  for(int iter = 0; iter < itol; iter++){    
    Mat C = P*Mat::ones(N,1,CV_32F)/N; normalize(C,C);
    if(iter > 0){if(norm(C,C_old) < ftol)break;}
    C_old = C.clone();
    for(int i = 0; i < N; i++){
      Mat R = this->rot_scale_align(P.col(i),C);
      for(int j = 0; j < n; j++){
    float x = P.fl(2*j,i),y = P.fl(2*j+1,i);
    P.fl(2*j  ,i) = R.fl(0,0)*x + R.fl(0,1)*y;
    P.fl(2*j+1,i) = R.fl(1,0)*x + R.fl(1,1)*y;
      }
    }
  }return P;
}
//=============================================================================
Mat
shape_model::
rot_scale_align(const Mat &src,
        const Mat &dst)
{
  //construct linear system
  int n = src.rows/2; float a=0,b=0,d=0;
  for(int i = 0; i < n; i++){
    d += src.fl(2*i) * src.fl(2*i  ) + src.fl(2*i+1) * src.fl(2*i+1);
    a += src.fl(2*i) * dst.fl(2*i  ) + src.fl(2*i+1) * dst.fl(2*i+1);
    b += src.fl(2*i) * dst.fl(2*i+1) - src.fl(2*i+1) * dst.fl(2*i  );
  }
  a /= d; b /= d;//solved linear system
  return (Mat_<float>(2,2) << a,-b,b,a);
}
//==============================================================================
Mat
shape_model::
calc_rigid_basis(const Mat &X)
{
  //compute mean shape
  int N = X.cols,n = X.rows/2;
  Mat mean = X*Mat::ones(N,1,CV_32F)/N;

  //construct basis for similarity transform
  Mat R(2*n,4,CV_32F);
  for(int i = 0; i < n; i++){
    R.fl(2*i,0) =  mean.fl(2*i  ); R.fl(2*i+1,0) =  mean.fl(2*i+1);
    R.fl(2*i,1) = -mean.fl(2*i+1); R.fl(2*i+1,1) =  mean.fl(2*i  );
    R.fl(2*i,2) =  1.0;            R.fl(2*i+1,2) =  0.0;
    R.fl(2*i,3) =  0.0;            R.fl(2*i+1,3) =  1.0;
  }
  //Gram-Schmidt orthonormalization
  for(int i = 0; i < 4; i++){
    Mat r = R.col(i);
    for(int j = 0; j < i; j++){
      Mat b = R.col(j); r -= b*(b.t()*r);
    }
    normalize(r,r);
  }return R;
}
//==============================================================================
void
shape_model::
clamp(const float c)
{
  double scale = p.fl(0);
  for(int i = 0; i < e.rows; i++){
    if(e.fl(i) < 0)continue;
    float v = c*sqrt(e.fl(i));
    if(fabs(p.fl(i)/scale) > v){
      if(p.fl(i) > 0)p.fl(i) =  v*scale;
      else           p.fl(i) = -v*scale;
    }
  }
}
//==============================================================================
void
shape_model::
write(FileStorage &fs) const
{
  assert(fs.isOpened()); 
  fs << "{" << "V"  << V << "e"  << e << "C" << C << "}";
}  
//==============================================================================
void
shape_model::
read(const FileNode& node)
{
  assert(node.type() == FileNode::MAP);
  node["V"] >> V; node["e"] >> e; node["C"] >> C;
  p = Mat::zeros(e.rows,1,CV_32F);
}
//==============================================================================
