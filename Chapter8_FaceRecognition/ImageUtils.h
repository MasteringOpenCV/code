/*****************************************************************************
*   Face Recognition using Eigenfaces or Fisherfaces
******************************************************************************
*   by Shervin Emami, 5th Dec 2012
*   http://www.shervinemami.info/openCV.html
******************************************************************************
*   Ch8 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

/*
 * ImageUtils
 *      Handy utility functions for dealing with images in OpenCV (desktop or Android).
 *      by Shervin Emami (shervin.emami@gmail.com), 27th May 2012.
 *      The most recent version of this will always be available from "http://shervinemami.info/openCV.html"
 */

#ifndef IMAGEUTILS_0_7_H_
#define IMAGEUTILS_0_7_H_


// OpenCV
#include <cv.h>
//#include <cvaux.h>
#include <cxcore.h>
#ifdef USE_HIGHGUI
    #include <highgui.h>
#endif


#include <stdio.h>
#if defined WIN32 || defined _WIN32
    #include <conio.h>        // For _kbhit() on Windows
    #include <direct.h>        // For mkdir(path) on Windows
    #define snprintf sprintf_s    // Visual Studio on Windows comes with sprintf_s() instead of snprintf()
#else
    #include <stdio.h>        // For getchar() on Linux
    #include <termios.h>    // For kbhit() on Linux
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>    // For mkdir(path, options) on Linux
#endif
#include <vector>
#include <string>
#include <iostream>            // for printing streams in C++
//#include <stdarg.h>        // for variable-length args in C
//#include <sstream>        // for printing floats in C++
//#include <fstream>        // for opening files in C++

// These functions will print using the LOG() function, using the same format as printf(). If you want it to be printed using a different
// function (such as for Android logcat output), then define LOG as your output function, otherwise it will use printf() by default.
#ifndef LOG
    // For stdout debug logging, with a new-line character on the end:
    #ifndef _MSC_VER
        // Compiles on GCC but not MSVC:
        #define LOG(fmt, args...) do {printf(fmt, ## args); printf("\n"); fflush(stdout);} while (0)
//        #define LOG printf
    #else
        #define LOG printf
    #endif

    //// For Android debug logging to logcat:
    //#include <android/log.h>
    //#define LOG(fmt, args...) (__android_log_print(ANDROID_LOG_INFO, "........", fmt, ## args))
#endif


#ifdef __cplusplus
extern "C"
{
#endif


// Allow 'UCHAR' variables in both C and C++ code.
#ifndef UCHAR
    typedef unsigned char UCHAR;
#endif

// Allow 'bool' variables in both C and C++ code.
#ifndef __cplusplus
    typedef int bool;
    #define true (1)
    #define false (0)
#endif

// Allow default args in C++ code, but explicit args in C code.
#ifdef __cplusplus
    #define DEFAULT(val) = val
#else
    #define DEFAULT(val)
#endif



//------------------------------------------------------------------------------
// Timer functions
//------------------------------------------------------------------------------
// Record the execution time of some code, in milliseconds. By Shervin Emami, 27th May 2012.
// eg:
//  DECLARE_TIMING(myTimer);
//    ...
//  START_TIMING(myTimer);
//    printf("A slow calc = %f\n", pow(1.2, 3.4) );
//  STOP_TIMING(myTimer);
//  AVERAGE_TIMING(myTimer);
#define DECLARE_TIMING(s)           int64 timeStart_##s; int64 timeDiff_##s; int64 timeTally_##s = 0; int64 countTally_##s = 0; double timeMin_##s = DBL_MAX; double timeMax_##s = 0; int64 timeEnd_##s;
#define START_TIMING(s)             timeStart_##s = cvGetTickCount()
#define STOP_TIMING(s)              do {    timeEnd_##s = cvGetTickCount(); timeDiff_##s = (timeEnd_##s - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++; timeMin_##s = MIN(timeMin_##s, timeDiff_##s); timeMax_##s = MAX(timeMax_##s, timeDiff_##s);    } while (0)
#define GET_TIMING(s)               (double)(0.001 * ( (double)timeDiff_##s / (double)cvGetTickFrequency() ))
#define GET_MIN_TIMING(s)           (double)(0.001 * ( (double)timeMin_##s / (double)cvGetTickFrequency() ))
#define GET_MAX_TIMING(s)           (double)(0.001 * ( (double)timeMax_##s / (double)cvGetTickFrequency() ))
#define GET_AVERAGE_TIMING(s)       (double)(countTally_##s ? 0.001 * ( (double)timeTally_##s / ((double)countTally_##s * cvGetTickFrequency()) ) : 0)
#define GET_TOTAL_TIMING(s)         (double)(0.001 * ( (double)timeTally_##s / ((double)cvGetTickFrequency()) ))
#define GET_TIMING_COUNT(s)         (int)(countTally_##s)
#define CLEAR_AVERAGE_TIMING(s)     do {    timeTally_##s = 0; countTally_##s = 0;     } while (0)
#define SHOW_TIMING(s, msg)         LOG("%s time:\t %dms\t (ave=%dms min=%dms max=%dms, across %d runs).", msg, cvRound(GET_TIMING(s)), cvRound(GET_AVERAGE_TIMING(s)), cvRound(GET_MIN_TIMING(s)), cvRound(GET_MAX_TIMING(s)), GET_TIMING_COUNT(s) )
#define SHOW_TOTAL_TIMING(s, msg)   LOG("%s total:\t %dms\t (ave=%dms min=%dms max=%dms, across %d runs).", msg, cvRound(GET_TOTAL_TIMING(s)), cvRound(GET_AVERAGE_TIMING(s)), cvRound(GET_MIN_TIMING(s)), cvRound(GET_MAX_TIMING(s)), GET_TIMING_COUNT(s) )
#define AVERAGE_TIMING(s)           SHOW_TIMING(s, #s)
#define TOTAL_TIMING(s)             do {    SHOW_TOTAL_TIMING(s, #s); CLEAR_AVERAGE_TIMING(s);     } while (0)


// Convert a float number to an int by rounding to nearest int using a certain method.
// Replace the code with the fastest method detected by 'testTiming_FloatConversion()'.
inline int roundFloat(float f);

//------------------------------------------------------------------------------
// Graphing functions
//------------------------------------------------------------------------------

// Draw the graph of an array of floats into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawFloatGraph(const float *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), float minV DEFAULT(0.0), float maxV DEFAULT(0.0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Draw the graph of an array of ints into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawIntGraph(const int *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Draw the graph of an array of uchars into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawUCharGraph(const uchar *arraySrc, int nArrayLength, IplImage *imageDst DEFAULT(0), int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0), char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

// Display a graph of the given float array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawFloatGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showFloatGraph(const char *name, const float *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Display a graph of the given int array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawIntGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showIntGraph(const char *name, const int *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Display a graph of the given unsigned char array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawUCharGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showUCharGraph(const char *name, const uchar *arraySrc, int nArrayLength, int delay_ms DEFAULT(500), IplImage *background DEFAULT(0));

// Simple helper function to easily view an image, with an optional pause.
void showImage(const IplImage *img, int delay_ms DEFAULT(0), char *name DEFAULT(0));

// Call 'setGraphColor(0)' to reset the colors that will be used for graphs.
void setGraphColor(int index DEFAULT(0));
// Specify the exact color that the next graph should be drawn as.
void setCustomGraphColor(int R, int B, int G);

//------------------------------------------------------------------------------
// Color conversion functions
//------------------------------------------------------------------------------

// Return a new image that is always greyscale, whether the input image was RGB or Greyscale.
// Remember to free the returned image using cvReleaseImage() when finished.
IplImage* convertImageToGreyscale(const IplImage *imageSrc);

// Create an RGB image from the YIQ image using an approximation of NTSC conversion(ref: "YIQ" Wikipedia page).
// Remember to free the generated RGB image.
IplImage* convertImageYIQtoRGB(const IplImage *imageYIQ);

// Create a YIQ image from the RGB image using an approximation of NTSC conversion(ref: "YIQ" Wikipedia page).
// Remember to free the generated YIQ image.
IplImage* convertImageRGBtoYIQ(const IplImage *imageRGB);

// Create an RGB image from the HSV image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated RGB image.
IplImage* convertImageHSVtoRGB(const IplImage *imageHSV);

// Create a HSV image from the RGB image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated HSV image.
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB);

// Do the color conversion of a single pixel, from RGB to HSV using Hue values between 0 to 255, whereas OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
inline void convertPixelRGBtoHSV_256(int bR, int bG, int bB, int &bH, int &bS, int &bV);
// Do the color conversion of a single pixel, from HSV to RGB using Hue values between 0 to 255, whereas OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
inline void convertPixelHSVtoRGB_256(int bH, int bS, int bV, int &bR, int &bG, int &bB);

// Do the color conversion of a single pixel, from RGB to HSV using Hue values between 0 to 179 just like OpenCV does, instead of Hues upto 255.
void convertPixelRGBtoHSV_180(int bR, int bG, int bB, int &bH, int &bS, int &bV);
// Do the color conversion of a single pixel, from HSV to RGB using Hue values between 0 to 179 just like OpenCV does, instead of Hues upto 255.
void convertPixelHSVtoRGB_180(int bH, int bS, int bV, int &bR, int &bG, int &bB);

//------------------------------------------------------------------------------
// 2D Point functions
//------------------------------------------------------------------------------
// Return (pointA + pointB).
CvPoint2D32f addPointF(const CvPoint2D32f pointA, const CvPoint2D32f pointB);
// Return (pointA - pointB).
CvPoint2D32f subtractPointF(const CvPoint2D32f pointA, const CvPoint2D32f pointB);

// Return (point * scale).
CvPoint2D32f scalePointF(const CvPoint2D32f point, float scale);
// Return the point scaled relative to the given origin.
CvPoint2D32f scalePointAroundPointF(const CvPoint2D32f point, const CvPoint2D32f origin, float scale);

// Return (p * s), to a maximum value of maxVal
float scaleValueF(float p, float s, float maxVal);
#define scaleValueFI(p, s, maxVal) scaleValueF(p, s, (float)maxVal)
// Return (p * s), to a maximum value of maxVal
int scaleValueI(int p, float s, int maxVal);

// Return the point rotated around its origin by angle (in degrees).
CvPoint2D32f rotatePointF(const CvPoint2D32f point, float angleDegrees);
// Return the point rotated around the given origin by angle (in degrees).
CvPoint2D32f rotatePointAroundPointF(const CvPoint2D32f point, const CvPoint2D32f origin, float angleDegrees);

// Calculate the distance between the 2 given (floating-point) points.
float findDistanceBetweenPointsF(const CvPoint2D32f p1, const CvPoint2D32f p2);
// Calculate the distance between the 2 given (integer) points.
float findDistanceBetweenPointsI(const CvPoint p1, const CvPoint p2);

// Calculate the angle between the 2 given integer points (in degrees).
float findAngleBetweenPointsF(const CvPoint2D32f p1, const CvPoint2D32f p2);
// Calculate the angle between the 2 given floating-point points (in degrees).
float findAngleBetweenPointsI(const CvPoint p1, const CvPoint p2);

// Draw a crossbar at the given position.
void drawCross(IplImage *img, const CvPoint pt, int radius, const CvScalar color );

// Print the label and then the rect to the console for easy debugging
void printPoint(const CvPoint pt, const char *label);
// Print the label and then the rect to the console for easy debugging
void printPointF(const CvPoint2D32f pt, const char *label);


//------------------------------------------------------------------------------
// Rectangle region functions
//------------------------------------------------------------------------------

// Enlarge or shrink the rectangle size & center by a given scale.
// If w & h are given, will make sure the rectangle stays within the bounds if it is enlarged too much.
// Note that for images, w should be (width-1) and h should be (height-1).
CvRect scaleRect(const CvRect rectIn, float scaleX, float scaleY, int w DEFAULT(0), int h DEFAULT(0));

// Enlarge or shrink the rectangle region by a given scale without moving its center, and possibly add a border around it.
// If w & h are given, will make sure the rectangle stays within the bounds if it is enlarged too much.
// Note that for images, w should be (width-1) and h should be (height-1).
CvRect scaleRectInPlace(const CvRect rectIn, float scaleX, float scaleY, float borderX DEFAULT(0.0f), float borderY DEFAULT(0.0f), int w DEFAULT(0), int h DEFAULT(0));

// Return a new rect that the same as rectA when shifted by (rectB.x, rectB.y).
CvRect offsetRect(const CvRect rectA, const CvRect rectB);
// Return a new rect that the same as rectA when shifted by (pt.x, pt.y).
CvRect offsetRectPt(const CvRect rectA, const CvPoint pt);

// Draw a rectangle around the given object. (Use CV_RGB(255,0,0) for red color)
void drawRect(IplImage *img, const CvRect rect, const CvScalar color DEFAULT(CV_RGB(220,0,0)));
// Draw a filled rectangle around the given object.
void drawRectFilled(IplImage *img, const CvRect rect, const CvScalar color DEFAULT(CV_RGB(220,0,0)));

// Print the label and then the rectangle information to the console for easy debugging
void printRect(const CvRect rect, const char *label DEFAULT(0));

// Make sure the given rectangle is completely within the given image dimensions.
CvRect cropRect(const CvRect rectIn, int w, int h);

//------------------------------------------------------------------------------
// Image transforming functions
//------------------------------------------------------------------------------

// Returns a new image that is a cropped version of the original image.
// Remember to free the new image later.
IplImage* cropImage(const IplImage *img, const CvRect region);

// Creates a new image copy that is of a desired size. The aspect ratio will be kept constant if desired, by cropping the desired region.
// Remember to free the new image later.
IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight, bool keepAspectRatio);

// Rotate the image clockwise and possibly scale the image. Use 'mapRotatedImagePoint()' to map pixels from the src to dst image.
IplImage *rotateImage(const IplImage *src, float angleDegrees, float scale DEFAULT(1.0f));

// Get the position of a pixel in the image after the rotateImage() operation.
CvPoint2D32f mapRotatedImagePoint(const CvPoint2D32f pointOrig, const IplImage *image, float angleRadians, float scale DEFAULT(1.0f));

//------------------------------------------------------------------------------
// Image utility functions
//------------------------------------------------------------------------------

// Paste multiple images next to each other as a single image, for saving or displaying.
// Remember to free the returned image.
// Sample usage: cvSaveImage("out.png", combineImages(2, img1, img2) );
// Modified by Shervin from the cvShowManyImages() function on the OpenCVWiki by Parameswaran.
// 'combineImagesResized()' will resize all images to 300x300, whereas 'combineImages()' doesn't resize the images at all.
IplImage* combineImagesResized(int nArgs, ...);
IplImage* combineImages(int nArgs, ...);

// Do Bilateral Filtering to smooth the image noise but preserve the edges.
// A smoothness of 5 is very little filtering, and 100 is very high filtering.
// Remember to free the returned image.
IplImage* smoothImageBilateral(const IplImage *src, float smoothness DEFAULT(30));

// Blend color images 'image1' and 'image2' using an 8-bit alpha-blending mask channel.
// Equivalent to this operation on each pixel: imageOut = image1 * (1-(imageAlphaMask/255)) + image2 * (imageAlphaMask/255)
// So if a pixel in imageAlphMask is 0, then that pixel in imageOut will be image1, or if imageAlphaMask is 255 then imageOut is image2,
// or if imageAlphaMask was 200 then imageOut would be: image1 * 0.78 + image2 * 0.22
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* blendImage(const IplImage* image1, const IplImage* image2, const IplImage* imageAlphaMask);

// Get an 8-bit equivalent of the 32-bit Float Matrix.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* convertMatrixToUcharImage(const CvMat *srcMat);

// Get a normal 8-bit uchar image equivalent of the 32-bit float greyscale image.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* convertFloatImageToUcharImage(const IplImage *srcImg);

// Save the given image to a JPG or BMP file, even if its format isn't an 8-bit image, such as a 32bit image.
int saveImage(const char *filename, const IplImage *image);

// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
// since cvSaveImage() can only handle 8bit images (not 32bit float images).
void saveFloatMat(const char *filename, const CvMat *src);

// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
// since cvSaveImage() can only handle 8bit images (not 32bit float images).
void saveFloatImage(const char *filename, const IplImage *srcImg);

// Draw some text onto an image using printf() format.
void drawText(IplImage *img, CvPoint position, CvScalar color, char *fmt, ...);

// Print the label and then some text info about the IplImage properties, to LOG() for easy debugging.
void printImageInfo(const IplImage *image_tile, const char *label DEFAULT(0));
// Print the pixel values of the IplImage, to LOG() for easy debugging. maxElements of 0 will print all the data.
void printImagePixels(const IplImage *image, const char *label DEFAULT(0), int maxElements DEFAULT(300));
// Print the label and then contents of a cv::Mat from the C++ interface (using "LOG()") for easy debugging.
void printMat(const cv::Mat M, const char *label DEFAULT(0), int maxElements DEFAULT(300));
// Print the label and then contents of the Matrix (using "LOG()") for easy debugging. maxElements of 0 will print all the data.
void printMatrix(const CvMat *M, const char *label DEFAULT(0), int maxElements DEFAULT(300));
// Print the label and info of a cv::Mat from the C++ interface (using "LOG()") for easy debugging.
void printMatInfo(const cv::Mat M, const char *label DEFAULT(0));
// Print the x & y coords of the given point. If 'label' is supplied, prints that first, and also prints a newline character on the end.
void printPoint32f(const CvPoint2D32f pt, const char *label DEFAULT(0));
// Print the start & end coords of the given line. If 'label' is supplied, prints that first, and also prints a newline character on the end.
void printLine(const CvPoint ptA, const CvPoint ptB, const char *label DEFAULT(0));
// Just for debugging float images & matrices.
void printDataRange(const CvArr *src, const char *msg);



#if defined (__cplusplus)
}
#endif


#endif /* IMAGEUTILS_H_ */
