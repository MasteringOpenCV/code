/*****************************************************************************
*   BGRAVideoFrame.h
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_BGRAVideoFrame_h
#define Example_MarkerBasedAR_BGRAVideoFrame_h

#include <cstddef>

// A helper struct presenting interleaved BGRA image in memory.
struct BGRAVideoFrame
{
    size_t width;
    size_t height;
    size_t stride;
    
    unsigned char * data;
};


#endif
