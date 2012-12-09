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

#include "OCR.h"

#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const int numFilesChars[]={35, 40, 42, 41, 42, 33, 30, 31, 49, 44, 30, 24, 21, 20, 34, 9, 10, 3, 11, 3, 15, 4, 9, 12, 10, 21, 18, 8, 15, 7};

int main ( int argc, char** argv )
{
    cout << "OpenCV Training OCR Automatic Number Plate Recognition\n";
    cout << "\n";

    char* path;
    
    //Check if user specify image to process
    if(argc >= 1 )
    {
        path= argv[1];
    
    }else{
        cout << "Usage:\n" << argv[0] << " <path to chars folders files> \n";
        return 0;
    }        






    Mat classes;
    Mat trainingDataf5;
    Mat trainingDataf10;
    Mat trainingDataf15;
    Mat trainingDataf20;

    vector<int> trainingLabels;
    OCR ocr;

    for(int i=0; i< OCR::numCharacters; i++)
    {
        int numFiles=numFilesChars[i];
        for(int j=0; j< numFiles; j++){
            cout << "Character "<< OCR::strCharacters[i] << " file: " << j << "\n";
            stringstream ss(stringstream::in | stringstream::out);
            ss << path << OCR::strCharacters[i] << "/" << j << ".jpg";
            Mat img=imread(ss.str(), 0);
            Mat f5=ocr.features(img, 5);
            Mat f10=ocr.features(img, 10);
            Mat f15=ocr.features(img, 15);
            Mat f20=ocr.features(img, 20);

            trainingDataf5.push_back(f5);
            trainingDataf10.push_back(f10);
            trainingDataf15.push_back(f15);
            trainingDataf20.push_back(f20);
            trainingLabels.push_back(i);
        }
    }

    
    trainingDataf5.convertTo(trainingDataf5, CV_32FC1);
    trainingDataf10.convertTo(trainingDataf10, CV_32FC1);
    trainingDataf15.convertTo(trainingDataf15, CV_32FC1);
    trainingDataf20.convertTo(trainingDataf20, CV_32FC1);
    Mat(trainingLabels).copyTo(classes);

    FileStorage fs("OCR.xml", FileStorage::WRITE);
    fs << "TrainingDataF5" << trainingDataf5;
    fs << "TrainingDataF10" << trainingDataf10;
    fs << "TrainingDataF15" << trainingDataf15;
    fs << "TrainingDataF20" << trainingDataf20;
    fs << "classes" << classes;
    fs.release();

    return 0;
}
