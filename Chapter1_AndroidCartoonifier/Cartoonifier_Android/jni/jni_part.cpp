/*****************************************************************************
*   Cartoonifier, for Android.
******************************************************************************
*   by Shervin Emami, 5th Dec 2012 (shervin.emami@gmail.com)
*   http://www.shervinemami.info/
******************************************************************************
*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "cartoon.h"
#include "ImageUtils.h" // Handy functions for debugging OpenCV images, by Shervin Emami.


using namespace std;
using namespace cv;

extern "C" {


// Just show the plain camera image without modifying it.
JNIEXPORT void JNICALL Java_com_Cartoonifier_CartoonifierView_ShowPreview(JNIEnv* env, jobject,
        jint width, jint height, jbyteArray yuv, jintArray bgra)
{
    // Get native access to the given Java arrays.
    jbyte* _yuv  = env->GetByteArrayElements(yuv, 0);
    jint*  _bgra = env->GetIntArrayElements(bgra, 0);

    // Prepare a cv::Mat that points to the YUV420sp data.
    Mat myuv(height + height/2, width, CV_8UC1, (uchar *)_yuv);
    // Prepare a cv::Mat that points to the BGRA output data.
    Mat mbgra(height, width, CV_8UC4, (uchar *)_bgra);

    // Convert the color format from the camera's
    // NV21 "YUV420sp" format to an Android BGRA color image.
    cvtColor(myuv, mbgra, CV_YUV420sp2BGRA);

    // OpenCV can now access/modify the BGRA image if we want ...


    // Release the native lock we placed on the Java arrays.
    env->ReleaseIntArrayElements(bgra, _bgra, 0);
    env->ReleaseByteArrayElements(yuv, _yuv, 0);
}


DECLARE_TIMING(CartoonifyImage);


// Modify the camera image using the Cartoonifier filter.
JNIEXPORT void JNICALL Java_com_Cartoonifier_CartoonifierView_CartoonifyImage(JNIEnv* env, jobject,
        jint width, jint height, jbyteArray yuv, jintArray bgra,
        jboolean sketchMode, jboolean alienMode, jboolean evilMode, jboolean debugMode)
{
    START_TIMING(CartoonifyImage);

    // Get native access to the given Java arrays.
    jbyte* _yuv  = env->GetByteArrayElements(yuv, 0);
    jint*  _bgra = env->GetIntArrayElements(bgra, 0);

    // Input color format (from camera):
    // "myuv" is the color image in the camera's native NV21 YUV 420 "semi-planar" format, which means
    // the first part of the array is the grayscale pixel array, followed by a quarter-sized pixel
    // array that is the U & V color channels interleaved. So if we just want to access a grayscale
    // image, we can get it directly from the 1st part of a YUV420sp semi-planar image without any
    // conversions. But if we want a color image (eg: BGRA color format that is recommended for OpenCV
    // on Android), then we must convert the color format using cvtColor().
    Mat myuv(height + height/2, width, CV_8UC1, (unsigned char *)_yuv); // Wrapper around the _yuv data.
    Mat mgray(height, width, CV_8UC1, (unsigned char *)_yuv); // Also a wrapper around the _yuv data.

    // Output color format (for display):
    // "mbgra" is the color image to be displayed on the Android device, in BGRA format (ie: OpenCV's
    // default BGR which is RGB but in the opposite byte order, but with an extra 0 byte on the end
    // of each pixel, so that each pixel is stored as Blue, Green, Red, 0). You can either do all
    // your processing in OpenCV's default BGR format and then convert your final output from BGR to
    // BGRA before display on the screen, or ideally you can ensure your image processing code can
    // handle BGRA format instead of or in addition to BGR format. This is particularly important if
    // you try to access pixels directly in the image!
    Mat mbgra(height, width, CV_8UC4, (unsigned char *)_bgra);

    // Convert the color format from the camera's YUV420sp semi-planar format to a regular BGR color image.
    Mat mbgr(height, width, CV_8UC3);   // Allocate a new image buffer.
    cvtColor(myuv, mbgr, CV_YUV420sp2BGR);

    
    //--- Beginning of custom C/C++ image processing with OpenCV ---//
    Mat displayedFrame(mbgra.size(), CV_8UC3);

    // Use debug type 1 (for mobile) if debug mode is enabled, since we can't show popup GUI windows.
    int debugType = 0;
    if (debugMode)
        debugType = 1;

    // Do the C/C++ image processing.
    cartoonifyImage(mbgr, displayedFrame, sketchMode, alienMode, evilMode, debugType);

    // Convert back from OpenCV's BGR format to Android's BGRA format, unless if we can handle BGRA in our code.
    cvtColor(displayedFrame, mbgra, CV_BGR2BGRA);
    //--- End of custom C/C++ image processing with OpenCV ---//

    
    // Release the native lock we placed on the Java arrays.
    env->ReleaseIntArrayElements(bgra, _bgra, 0);
    env->ReleaseByteArrayElements(yuv, _yuv, 0);

    STOP_TIMING(CartoonifyImage);
    // Print the timing info.
    SHOW_TIMING(CartoonifyImage, "CartoonifyImage");
}



}//end of extern "C" (global C/C++ functions that aren't part of a C++ Class)
