/*****************************************************************************
*   Number Plate Recognition using SVM and Neural Networks
******************************************************************************
*   by David Millán Escrivá, 5th Dec 2012
*   http://blog.damiles.com
******************************************************************************
*   Ch5 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

// Main entry code OpenCV

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
    cout << "OpenCV Training SVM Automatic Number Plate Recognition\n";
    cout << "\n";

    char* path_Plates;
    char* path_NoPlates;
    int numPlates;
    int numNoPlates;
    int imageWidth=144;
    int imageHeight=33;

    //Check if user specify image to process
    if(argc >= 5 )
    {
        numPlates= atoi(argv[1]);
        numNoPlates= atoi(argv[2]);
        path_Plates= argv[3];
        path_NoPlates= argv[4];

    }else{
        cout << "Usage:\n" << argv[0] << " <num Plate Files> <num Non Plate Files> <path to plate folder files> <path to non plate files> \n";
        return 0;
    }        

    Mat classes;//(numPlates+numNoPlates, 1, CV_32FC1);
    Mat trainingData;//(numPlates+numNoPlates, imageWidth*imageHeight, CV_32FC1 );

    Mat trainingImages;
    vector<int> trainingLabels;

    for(int i=0; i< numPlates; i++)
    {

        stringstream ss(stringstream::in | stringstream::out);
        ss << path_Plates << i << ".jpg";
        Mat img=imread(ss.str(), 0);
        img= img.reshape(1, 1);
        trainingImages.push_back(img);
        trainingLabels.push_back(1);
    }

    for(int i=0; i< numNoPlates; i++)
    {
        stringstream ss(stringstream::in | stringstream::out);
        ss << path_NoPlates << i << ".jpg";
        Mat img=imread(ss.str(), 0);
        img= img.reshape(1, 1);
        trainingImages.push_back(img);
        trainingLabels.push_back(0);

    }

    Mat(trainingImages).copyTo(trainingData);
    //trainingData = trainingData.reshape(1,trainingData.rows);
    trainingData.convertTo(trainingData, CV_32FC1);
    Mat(trainingLabels).copyTo(classes);

    FileStorage fs("SVM.xml", FileStorage::WRITE);
    fs << "TrainingData" << trainingData;
    fs << "classes" << classes;
    fs.release();

    return 0;
}
