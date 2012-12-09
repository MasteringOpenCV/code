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
  visualize_annotations: Display annotated data to screen
  Jason Saragih (2012)
*/
#include "opencv_hotshots/ft/ft.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
//==============================================================================
int main(int argc,char** argv)
{
  //load data
  if(argc < 2){
    cout << "usage: ./visualise_annotation annotation_file" << endl; 
    return 0;
  }
  ft_data data = load_ft<ft_data>(argv[1]);
  if(data.imnames.size() == 0){
    cerr << "Data file does not contain any annotations."<< endl; return 0;
  }
  data.rm_incomplete_samples();
  cout << "n images: " << data.imnames.size() << endl
       << "n points: " << data.symmetry.size() << endl
       << "n connections: " << data.connections.size() << endl;

  //display data
  namedWindow("Annotations");
  int index = 0; bool flipped = false;
  while(1){
    Mat image;
    if(flipped)image = data.get_image(index,3);
    else image = data.get_image(index,2);
    data.draw_connect(image,index,flipped);
    data.draw_sym(image,index,flipped);
    imshow("Annotations",image);
    int c = waitKey(0);
    if(c == 'q')break;
    else if(c == 'p')index++;
    else if(c == 'o')index--;
    else if(c == 'f')flipped = !flipped;
    if(index < 0)index = 0;
    else if(index >= int(data.imnames.size()))index = data.imnames.size()-1;
  }
  destroyWindow("Annotations"); return 0;
}
//==============================================================================
