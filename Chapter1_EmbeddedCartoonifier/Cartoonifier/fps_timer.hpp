/*****************************************************************************
*   Simple FPS event timer
******************************************************************************
*   by Jason Saragih, 5th Dec 2012
*   http://jsaragih.org/
******************************************************************************
*   Ch6 of the book "Mastering OpenCV with Practical Computer Vision Projects", 2nd Edition
*   Copyright Packt Publishing 2016.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************
*   (Code was re-formatted and re-used by Shervin Emami for Ch1 of the same book)
*****************************************************************************/

#pragma once


#include <iostream>
#include "stdio.h"      // For 'sprintf()'


//==============================================================================
class fps_timer {                            //frames/second timer for tracking
public:
    int64 t_start;                           //start time
    int64 t_end;                             //end time
    float fps;                               //current frames/sec
    int fnum;                                //number of frames since @t_start

    fps_timer() {                            //default constructor
        this->reset();
    }

    void increment() {                       //increment timer index
        if(fnum >= 29) {
            t_end = cv::getTickCount();
            fps = 30.0 / (float(t_end-t_start)/getTickFrequency());
            t_start = t_end; fnum = 0;
        } else
            fnum += 1;
    }


    void reset() {                           //reset timer
        t_start = cv::getTickCount();
        fps = 0;
        fnum = 0;
    }


    void display_fps(Mat &im,                     //image to display FPS on
                     Point p = Point(-1,-1)) {    //bottom left corner of text
        char str[256];
        Point pt;
        if (p.y < 0)
            pt = Point(10,im.rows-20);
        else
            pt = p;
        sprintf(str,"%d frames/sec", (int)cvRound(fps));
        string text = str;
        putText(im,text,pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar::all(255));
    }

};
