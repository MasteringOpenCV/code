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

#ifndef OCR_h
#define OCR_h

#include <string.h>
#include <vector>

#include "Plate.h"

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <ml.h>

using namespace std;
using namespace cv;


#define HORIZONTAL    1
#define VERTICAL    0

class CharSegment{
public:
    CharSegment();
    CharSegment(Mat i, Rect p);
    Mat img;
    Rect pos;
};

class OCR{
    public:
        bool DEBUG;
        bool saveSegments;
        string filename;
        static const int numCharacters;
        static const char strCharacters[];
        OCR(string trainFile);
        OCR();
        string run(Plate *input);
        int charSize;
        Mat preprocessChar(Mat in);
        int classify(Mat f);
        void train(Mat trainData, Mat trainClasses, int nlayers);
        int classifyKnn(Mat f);
        void trainKnn(Mat trainSamples, Mat trainClasses, int k);
        Mat features(Mat input, int size);

    private:
        bool trained;
        vector<CharSegment> segment(Plate input);
        Mat Preprocess(Mat in, int newSize);        
        Mat getVisualHistogram(Mat *hist, int type);
        void drawVisualFeatures(Mat character, Mat hhist, Mat vhist, Mat lowData);
        Mat ProjectedHistogram(Mat img, int t);
        bool verifySizes(Mat r);
        CvANN_MLP  ann;
        CvKNearest knnClassifier;
        int K;
};

#endif
