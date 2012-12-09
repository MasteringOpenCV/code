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

#include "Plate.h"

Plate::Plate(){
}

Plate::Plate(Mat img, Rect pos){
    plateImg=img;
    position=pos;
}

string Plate::str(){
    string result="";
    //Order numbers
    vector<int> orderIndex;
    vector<int> xpositions;
    for(int i=0; i< charsPos.size(); i++){
        orderIndex.push_back(i);
        xpositions.push_back(charsPos[i].x);
    }
    float min=xpositions[0];
    int minIdx=0;
    for(int i=0; i< xpositions.size(); i++){
        min=xpositions[i];
        minIdx=i;
        for(int j=i; j<xpositions.size(); j++){
            if(xpositions[j]<min){
                min=xpositions[j];
                minIdx=j;
            }
        }
        int aux_i=orderIndex[i];
        int aux_min=orderIndex[minIdx];
        orderIndex[i]=aux_min;
        orderIndex[minIdx]=aux_i;
        
        float aux_xi=xpositions[i];
        float aux_xmin=xpositions[minIdx];
        xpositions[i]=aux_xmin;
        xpositions[minIdx]=aux_xi;
    }
    for(int i=0; i<orderIndex.size(); i++){
        result=result+chars[orderIndex[i]];
    }
    return result;
}
