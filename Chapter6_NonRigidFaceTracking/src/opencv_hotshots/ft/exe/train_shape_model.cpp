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
  train_shape_model: Learn a shape_model object from training data
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
const char* usage = 
  "usage: ./train_shape_model annotation_file shape_model_file " 
  "[-f fraction_of_variation] [-k maximum_modes] [--mirror]";
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
float
parse_frac(int argc,char** argv)
{
  for(int i = 1; i < argc; i++){
    string str = argv[i];
    if(str.length() != 2)continue;
    if(strcmp(str.c_str(),"-f") == 0){
      if(argc > i+1)return atof(argv[i+1]);
    }
  }return 0.95;
}
//==============================================================================
float
parse_kmax(int argc,char** argv)
{
  for(int i = 1; i < argc; i++){
    string str = argv[i];
    if(str.length() != 2)continue;
    if(strcmp(str.c_str(),"-k") == 0){
      if(argc > i+1)return atoi(argv[i+1]);
    }
  }return 20;
}
//==============================================================================
bool
parse_mirror(int argc,char** argv)
{
  for(int i = 1; i < argc; i++){
    string str = argv[i];
    if(str.length() != 8)continue;
    if(strcmp(str.c_str(),"--mirror") == 0)return true;
  }return false;
}
//==============================================================================
int main(int argc,char** argv)
{
  //load data
  if(argc < 3){ cout << usage << endl; return 0;}
  float frac = parse_frac(argc,argv);
  int kmax = parse_kmax(argc,argv);
  bool mirror = parse_mirror(argc,argv);
  ft_data data = load_ft<ft_data>(argv[1]);
  if(data.imnames.size() == 0){
    cerr << "Data file does not contain any annotations."<< endl; return 0;
  }
  //remove unlabeled samples and get reflections as well
  data.rm_incomplete_samples();
  vector<vector<Point2f> > points;
  for(int i = 0; i < int(data.points.size()); i++){
    points.push_back(data.get_points(i,false));
    if(mirror)points.push_back(data.get_points(i,true));
  }
  //train model and save to file
  cout << "shape model training samples: " << points.size() << endl;
  shape_model smodel; smodel.train(points,data.connections,frac,kmax);  
  cout << "retained: " << smodel.V.cols-4 << " modes" << endl;
  save_ft(argv[2],smodel); return 0;
}
//==============================================================================
