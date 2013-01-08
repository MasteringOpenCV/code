/*****************************************************************************
*   Number Plate Recognition using SVM and Neural Networks
******************************************************************************
*   by David Mill�n Escriv�, 5th Dec 2012
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
OCR ocr;

void generateRandom(int n, int min, int max, vector<int> *samples){
    int range=max-min;
    int r=rand()%range+min;
    if(samples->at(r)==0){
        samples->at(r)=1;
        n++;
    }
    if(n<100){
        generateRandom(n,min,max, samples);
    }
}

float test(Mat samples, Mat classes){
    float errors=0;
    for(int i=0; i<samples.rows; i++){
        
        int result= ocr.classify(samples.row(i));

        if(result!= classes.at<int>(i))
            errors++;
    }
    return errors/samples.rows;
}

int main ( int argc, char** argv )
{

    int nh;
    char* data;
    //Check params
    if(argc >= 2 )
    {
        nh= atoi(argv[1]);
        data= argv[2];
    }else{
        cout << "Usage:\n" << argv[0] << " <num hidden nodes> <data to load from xml>\n";
        return 0;
    }    
    
    Mat classes;
    Mat trainingData;
    
    //Read file storage.
    FileStorage fs;
    fs.open("OCR.xml", FileStorage::READ);
    
    fs[data] >> trainingData;
    fs["classes"] >> classes;

    float result;
    
    //init random generator
    srand(time(NULL));
    //Create 100 random pos for samples
    std::vector<int> isSample(trainingData.rows,0);
    generateRandom(0,0,trainingData.rows-1,&isSample);

    //Create sample data
    Mat train, trainClasses;
    Mat samples, samplesClasses;
    for(int i=0; i<trainingData.rows; i++){
        if(isSample[i]==1){
            samples.push_back(trainingData.row(i));
            samplesClasses.push_back(classes.row(i));
        }else{
            train.push_back(trainingData.row(i));
            trainClasses.push_back(classes.row(i));
            
        }
    }
    
    result= 0;
    
    ocr.train(train, trainClasses, nh);
    result=test(samples, samplesClasses);
    
    //cout << nh <<  "\t" << data << "\t"<< result <<"\n";
    cout << result ;

    return 0;
}
