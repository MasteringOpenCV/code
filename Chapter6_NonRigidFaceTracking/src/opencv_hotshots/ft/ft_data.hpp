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
#ifndef _FT_FT_DATA_HPP_
#define _FT_FT_DATA_HPP_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace cv;
using namespace std;
//==============================================================================
class ft_data{                             //face tracker data
public:
  vector<int> symmetry;                    //indices of symmetric points
  vector<Vec2i> connections;               //indices of connected points 
  vector<string> imnames;                  //images
  vector<vector<Point2f> > points;         //points
  
  inline int n_images(){return imnames.size();}

  Mat                                      //idx'th image
  get_image(const int idx,                 //index of image to get
        const int flag = 2);           //0=gray,1=gray+flip,2=rgb,3=rgb+flip

  vector<Point2f>                          //points for idx'th image
  get_points(const int idx,                //index of image
         const bool flipped = false);  //flip points?

  void                                     //removes samples which have missing
  rm_incomplete_samples();                 //point annotations
  
  void
  rm_sample(const int idx);                //remove idx'th sample

  void
  draw_points(Mat &im,                     //image to draw on
          const int idx,               //index of shape 
          const bool flipped = false,  //flip points?
          const Scalar color=CV_RGB(255,0,0),//color to draw points in
          const vector<int> &pts=vector<int>());//indices of points to draw

  void
  draw_sym(Mat &im,                        //image to draw on
       const int idx,                  //index of shape 
       const bool flipped = false,     //flip points?
       const vector<int> &pts=vector<int>());//indices of points to draw

  void
  draw_connect(Mat &im,                    //image to draw on
           const int idx,              //index of shape 
           const bool flipped = false, //flip points?
           const Scalar color=CV_RGB(0,0,255),//color to draw points in
           const vector<int> &con=vector<int>());//indices of connections
  
  void 
  write(FileStorage &fs) const;            //file storage object to write to

  void 
  read(const FileNode& node);              //file storage node to read from
};
//==============================================================================
#endif
