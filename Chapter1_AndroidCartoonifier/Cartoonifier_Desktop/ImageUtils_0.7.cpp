/*****************************************************************************
*   ImageUtils
*   Handy utility functions for dealing with images in OpenCV (desktop or Android).
*   The most recent version of this will always be available from "http://shervinemami.info/openCV.html"
******************************************************************************
*   by Shervin Emami, 5th Dec 2012 (shervin.emami@gmail.com)
*   http://www.shervinemami.info/
******************************************************************************
*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#define USE_HIGHGUI        // Enable this to display graph windows using OpenCV's HighGUI. (Supports Windows, Linux & Mac, but not iPhone).

#include "ImageUtils.h"



using namespace std;


// Print the label and then some text info about the IplImage properties, to LOG() for easy debugging.
void printImageInfo(const IplImage *image, const char *label)
{
    string s = "";
    char buff[1024];
    if (label)
        s = label + string(": ");
    if (image) {
        snprintf(buff, sizeof(buff), "[Image] = %dw%dh, %d channels of %dbit depth, widthStep=%d, origin=%d", image->width, image->height, image->nChannels, image->depth, image->widthStep, image->origin);
        s += buff;
        if (image->roi)
            snprintf(buff, sizeof(buff), " ROI=[at %d,%d of size %dx%d, COI=%d].\n", image->roi->xOffset, image->roi->yOffset, image->roi->width, image->roi->height, image->roi->coi);
        else
            snprintf(buff, sizeof(buff), " ROI=<null>.\n");
        s += buff;
    }
    else {
        s = "[Image] = <null>\n";
    }
    // LOG can be printf or similar.
    LOG(s.c_str());
}

// Print the pixel values of the IplImage, to LOG() for easy debugging.
void printImagePixels(const IplImage *image, const char *label, int maxElements )
{
    string s;
    char buff[32];
    if (label)
        s = label + string(": ");
    else
        s = "Image: ";
    if (image) {
        if (maxElements == 0)
            maxElements = image->width * image->height;
        sprintf(buff, "(%dw%dh):\n", image->width, image->height);
        s += string(buff);
        LOG(s.c_str());

        int totalElements = 0;
        int depth = image->depth & 255;
        for (int row=0; row < image->height; row++) {
            string s = "";
            int element = 0;
            if (image->nChannels > 1 && image->height > 1) {
                snprintf(buff, sizeof(buff), "row%d: ", row);
            }
            for (int col=0; col < image->width; col++) {
                if (image->nChannels > 1)
                    s += "[";
                for (int ch=0; ch <= image->nChannels-1; ch++) {
                    if (ch > 0 || (image->nChannels == 1 && col != 0))    // Add a separator, except for the first element of this pixel or row.
                        s += ",";

                    // Allow to print just part of the image.
                    totalElements++;
                    if (totalElements > maxElements) {
                        // LOG can be printf or similar.
                        LOG("%s ... <just displaying the 1st %d entries from %d!>", s.c_str(), maxElements, image->width * image->height * image->nChannels);
                        return;
                    }

                    if (depth == 8)         // 8-bit UCHAR image.
                        snprintf(buff, sizeof(buff), "%d", image->imageData[(row * (image->widthStep)) + (col*image->nChannels) + ch]);
                    else if (depth == 16)   // 16-bit short image.
                        snprintf(buff, sizeof(buff), "%d", *(short*)(uchar*)&image->imageData[(row * image->widthStep) + ((col*image->nChannels) + ch) * sizeof(short)]);
                    else if (depth == 32)   // 32-bit float image.
                        snprintf(buff, sizeof(buff), "%.3f", *(float*)(uchar*)&image->imageData[(row * image->widthStep) + ((col*image->nChannels) + ch) * sizeof(float)]);
                    else if (depth == 64)   // 64-bit double image.
                        snprintf(buff, sizeof(buff), "%.4lf", *(double*)(uchar*)&image->imageData[(row * image->widthStep) + ((col*image->nChannels) + ch) * sizeof(double)]);
                    s += buff;

                    const int MAX_ELEMENTS_PER_LINE = 60;   // Only print upto 30 numbers per LOG() statement, since Android can only handle about 250 characters per log line!
                    if (element > MAX_ELEMENTS_PER_LINE) {
                        // LOG can be printf or similar.
                        LOG(s.c_str());
                        s = "";
                        element = 0;
                    }
                    element++;
                }
                if (image->nChannels > 1)
                    s += "] ";
            }
            // LOG can be printf or similar.
            LOG(s.c_str());
        }
    }
    else {
        LOG("[%s] = <null>!", s.c_str());
    }
}

// Return the number of bits in each channel of the given Mat. ie: 8, 16, 32 or 64.
int getBitDepth(const cv::Mat M)
{
    switch (CV_MAT_DEPTH(M.type())) {
        case CV_8U:
        case CV_8S:
            return 8;
        case CV_16U:
        case CV_16S:
            return 16;
        case CV_32S:
        case CV_32F:
            return 32;
        case CV_64F:
            return 64;
    }
    return -1;
}

// Print the contents of a multi-channel array (using "LOG()") for easy debugging.
// If 'maxElements' is 0, it will print the whole array. If it is -1, it will not print the array at all.
void printArray2D(const uchar *data, int cols, int rows, int channels, int depth_type, int step, int maxElements)
{
    char buff[32];
    if (data != 0 && cols > 0 && rows > 0 && channels > 0 && step > 0) {

        // Show the actual data values
        if (maxElements >= 0) {
            if (maxElements == 0)
                maxElements = rows * cols;
            int totalElements = 0;
            //int step = step;

            for (int row=0; row < rows; row++) {
                string s = "";
                int element = 0;
                if (channels > 1 && rows > 1) {
                    snprintf(buff, sizeof(buff), "row%d: ", row);
                }
                for (int col=0; col < cols; col++) {
                    if (channels > 1)
                        s += "[";
                    for (int ch=0; ch <= channels-1; ch++) {
                        if (ch > 0 || (channels == 1 && col != 0))    // Add a separator, except for the first element of this pixel or row.
                            s += ",";

                        buff[0] = '?';  // Initialize the string to "?" if something goes wrong.
                        buff[1] = 0;

                        // Allow to print just part of the image.
                        totalElements++;
                        if (totalElements > maxElements) {
                            // LOG can be printf or similar.
                            LOG("%s ... <just displaying the 1st %d entries from %d!>", s.c_str(), maxElements, rows * cols * channels);
                            return;
                        }

                        switch (depth_type) {
                        case CV_8U:
                        case CV_8S:         // 8-bit UCHAR Mat.
                            snprintf(buff, sizeof(buff), "%d", data[(row * step) + (col*channels) + ch]);
                            break;
                        case CV_16U:
                        case CV_16S:   // 16-bit short Mat.
                            snprintf(buff, sizeof(buff), "%d", *(short*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(short)]);
                            break;
                        case CV_32S:   // 32-bit int Mat.
                            snprintf(buff, sizeof(buff), "%d", *(int*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(int)]);
                            break;
                        case CV_32F:   // 32-bit float Mat.
                            snprintf(buff, sizeof(buff), "%.3f", *(float*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(float)]);
                            break;
                        case CV_64F:   // 64-bit double Mat.
                            snprintf(buff, sizeof(buff), "%.3lg", *(double*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(double)]);
                            break;
                        default:
                            snprintf(buff, sizeof(buff), "UNKNOWN DEPTH OF %d!", depth_type);
                        }
                        s += buff;

                        const int MAX_ELEMENTS_PER_LINE = 30;   // Only print upto 30 numbers per LOG() statement, since Android can only handle about 250 characters per log line!
                        if (element > MAX_ELEMENTS_PER_LINE) {
                            // LOG can be printf or similar.
                            LOG(s.c_str());
                            s = "";
                            element = 0;
                        }
                        element++;
                    }
                    if (channels > 1)
                        s += "] ";
                }
                // LOG can be printf or similar.
                LOG(s.c_str());
            }
        }//end if (maxElements>=0)
    }
}

// Print the label and then contents of a cv::Mat from the C++ interface (using "LOG()") for easy debugging.
// If 'maxElements' is 0, it will print the whole array. If it is -1, it will not print the array at all.
void printMat(const cv::Mat M, const char *label, int maxElements)
{
    string s;
    char buff[32];
    if (label)
        s = label + string(": ");
    else
        s = "Mat: ";
    if (!M.empty()) {
        int channels = CV_MAT_CN(M.type());
        int depth_bpp = getBitDepth(M);     // eg: 8, 16, 32.
        int depth_type = CV_MAT_DEPTH(M.type());    // eg: CV_32S, CV_32F

        // Show the dimensions & data type
        sprintf(buff, "%dw%dh %dch %dbpp", M.cols, M.rows, channels, depth_bpp);
        s += string(buff);

        // Show the data range for each channel
        s += ", range";
        for (int ch=0; ch<channels; ch++) {
            cv::Mat arr = cv::Mat(M.rows, M.cols, depth_type);
            // Extract one channel at a time, to show it's range.
            int from_to[2];
            from_to[0] = ch;
            from_to[1] = 0;
            cv::mixChannels( &M, 1, &arr, 1, from_to, 1 );
            // Show it's range.
            double minVal, maxVal;
            cv::minMaxLoc(arr, &minVal, &maxVal);
            snprintf(buff, sizeof(buff), "[%lg,%lg]", minVal, maxVal);
            s += buff;
        }
        LOG(s.c_str());

        // Show the actual data values
        printArray2D(M.data, M.cols, M.rows, channels, depth_type, M.step, maxElements);
    }
    else {
        LOG("%s empty Mat", s.c_str());
    }
}

// Print the label and info of a cv::Mat from the C++ interface (using "LOG()") for easy debugging.
void printMatInfo(const cv::Mat M, const char *label)
{
    printMat(M, label, -1);
}


// Print the label and then contents of a cvMat from the C interface (using "LOG()") for easy debugging.
void printMatrix(const CvMat *M, const char *label, int maxElements)
{
    string s;
    char buff[32];
    if (label)
        s = label + string(": ");
    else
        s = "Matrix: ";
    if (M) {
        if (maxElements == 0)
            maxElements = M->rows * M->cols;
        sprintf(buff, "[%drows x %dcols]:\n", M->rows, M->cols);
        s += string(buff);
        LOG(s.c_str());
        int channels = CV_MAT_CN(M->type);
        int depth = CV_MAT_DEPTH(M->type);
        int totalElements = 0;
        uchar *data = (uchar*)M->data.ptr;
        int step = M->step;

        for (int row=0; row < M->rows; row++) {
            string s = "";
            int element = 0;
            if (channels > 1 && M->rows > 1) {
                snprintf(buff, sizeof(buff), "row%d: ", row);
            }
            for (int col=0; col < M->cols; col++) {
                if (channels > 1)
                    s += "[";
                for (int ch=0; ch <= channels-1; ch++) {
                    if (ch > 0 || (channels == 1 && col != 0))    // Add a separator, except for the first element of this pixel or row.
                        s += ",";

                    buff[0] = '?';  // Initialize the string to "?" if something goes wrong.
                    buff[1] = 0;

                    // Allow to print just part of the image.
                    totalElements++;
                    if (totalElements > maxElements) {
                        // LOG can be printf or similar.
                        LOG("%s ... <just displaying the 1st %d entries from %d!>", s.c_str(), maxElements, M->rows * M->cols * channels);
                        return;
                    }

                    switch (depth) {
                    case CV_8U:
                    case CV_8S:         // 8-bit UCHAR Mat.
                        buff[0] = 'C';
                        //snprintf(buff, sizeof(buff), "%d", data[(row * step) + (col*channels) + ch]);
                        ; // UNTESTED!
                        break;
                    case CV_16U:
                    case CV_16S:   // 16-bit short Mat.
                        buff[0] = 'S';
                        //snprintf(buff, sizeof(buff), "%d", *(short*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(short)]);
                        ; // UNTESTED!
                        break;
                    case CV_32S:   // 32-bit int Mat.
                        buff[0] = 'I';
                        snprintf(buff, sizeof(buff), "%d", *(int*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(int)]);
                        ; // UNTESTED!
                        break;
                    case CV_32F:   // 32-bit float Mat.
                        snprintf(buff, sizeof(buff), "%.3f", *(float*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(float)]);
                        break;
                    case CV_64F:   // 64-bit double Mat.
                        buff[0] = 'D';
                        //snprintf(buff, sizeof(buff), "%.4lf", *(double*)(uchar*)&data[(row * step) + ((col*channels) + ch) * sizeof(double)]);
                        ; // UNTESTED!
                        break;
                    default:
                        snprintf(buff, sizeof(buff), "UNKNOWN DEPTH OF %d!", depth);
                    }
                    s += buff;

                    const int MAX_ELEMENTS_PER_LINE = 60;   // Only print upto 30 numbers per LOG() statement, since Android can only handle about 250 characters per log line!
                    if (element > MAX_ELEMENTS_PER_LINE) {
                        // LOG can be printf or similar.
                        LOG(s.c_str());
                        s = "";
                        element = 0;
                    }
                    element++;
                }
                if (channels > 1)
                    s += "] ";
            }
            // LOG can be printf or similar.
            LOG(s.c_str());
        }
    }
    else {
        LOG("[%s] = <null>!", s.c_str());
    }
}

// Print the x & y coords of the given point. If 'label' is supplied, prints that first, and also prints a newline character on the end.
void printPoint32f(const CvPoint2D32f pt, const char *label)
{
    if (label)
        cout << label << ": ";
    cout << "(" << pt.x << ", " << pt.y << ")";
    if (label)
        cout << endl;
}

// Print the start & end coords of the given line. If 'label' is supplied, prints that first, and also prints a newline character on the end.
void printLine(const CvPoint ptA, const CvPoint ptB, const char *label)
{
    if (label)
        cout << label << ": ";
    cout << "(" << ptA.x << ", " << ptA.y << ")-(" << ptB.x << ", " << ptB.y << ")";
    if (label)
        cout << endl;
}

// Just for debugging float images & matrices.
void printDataRange(const CvArr *src, const char *msg)
{
    if (((IplImage*)src)->nChannels == 2) {  // 2-ch (Complex) input
        double min_val[2] = {0};
        double max_val[2] = {0};
        IplImage * imgComplexSrcRe = cvCreateImage( cvGetSize(((IplImage*)src)), IPL_DEPTH_32F, 1 );
        IplImage * imgComplexSrcIm = cvCreateImage( cvGetSize(((IplImage*)src)), IPL_DEPTH_32F, 1 );
        cvSplit(src, imgComplexSrcRe, imgComplexSrcIm, NULL, NULL);
        cvMinMaxLoc(imgComplexSrcRe, &min_val[0], &max_val[0], NULL, NULL, NULL);
        cvMinMaxLoc(imgComplexSrcIm, &min_val[1], &max_val[1], NULL, NULL, NULL);
        LOG("\t`` %s Range: Complex MIN = %lf %lfj, MAX = %lf %lfj", msg, min_val[0], min_val[1], max_val[0], max_val[1]);
        cvReleaseImage(&imgComplexSrcRe);
        cvReleaseImage(&imgComplexSrcIm);
    }
    else if (((IplImage*)src)->nChannels == 1) {  // 1-ch (Real) input
        double min_val[1] = {0};
        double max_val[1] = {0};
        cvMinMaxLoc(src, &min_val[0], &max_val[0], NULL, NULL, NULL);
        LOG("\t`` %s Range: Real MIN = %lf, MAX = %lf", msg, min_val[0], max_val[0]);
    }
    else {
        LOG("\t`` %s Range: UNKNOWN because nChannels == %d != 1 or 2", msg, ((IplImage*)src)->nChannels);
    }
}

//------------------------------------------------------------------------------
// Graphing functions
//------------------------------------------------------------------------------
const CvScalar BLACK = CV_RGB(0,0,0);
const CvScalar WHITE = CV_RGB(255,255,255);
const CvScalar GREY = CV_RGB(150,150,150);

int countGraph = 0;    // Used by 'getGraphColor()'
CvScalar customGraphColor;
int usingCustomGraphColor = 0;

// Get a new color to draw graphs. Will use the latest custom color, or change between blue, green, red, dark-blue, dark-green and dark-red until a new image is created.
CvScalar getGraphColor(void)
{
    if (usingCustomGraphColor) {
        usingCustomGraphColor = 0;
        return customGraphColor;
    }

    countGraph++;
    switch (countGraph) {
    case 1:    return CV_RGB(60,60,255);    // light-blue
    case 2:    return CV_RGB(60,255,60);    // light-green
    case 3:    return CV_RGB(255,60,40);    // light-red
    case 4:    return CV_RGB(0,210,210);    // blue-green
    case 5:    return CV_RGB(180,210,0);    // red-green
    case 6:    return CV_RGB(210,0,180);    // red-blue
    case 7:    return CV_RGB(0,0,185);        // dark-blue
    case 8:    return CV_RGB(0,185,0);        // dark-green
    case 9:    return CV_RGB(185,0,0);        // dark-red
    default:
        countGraph = 0;    // start rotating through colors again.
        return CV_RGB(200,200,200);    // grey
    }
}
// Call 'setGraphColor()' to reset the colors that will be used for graphs.
void setGraphColor(int index)
{
    countGraph = index;
    usingCustomGraphColor = 0;    // dont use a custom color.
}
// Specify the exact color that the next graph should be drawn as.
void setCustomGraphColor(int R, int B, int G)
{
    customGraphColor = CV_RGB(R, G, B);
    usingCustomGraphColor = 1;    // show that it will be used.
}

// Draw the graph of an array of floats into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawFloatGraph(const float *arraySrc, int nArrayLength, IplImage *imageDst, float minV, float maxV, int width, int height, char *graphLabel, bool showScale)
{
    int w = width;
    int h = height;
    int b = 10;        // border around graph within the image
    if (w <= 20)
        w = nArrayLength + b*2;    // width of the image
    if (h <= 20)
        h = 220;

    int s = h - b*2;// size of graph height
    float xscale = 1.0;
    if (nArrayLength > 1)
        xscale = (w - b*2) / (float)(nArrayLength-1);    // horizontal scale
    IplImage *imageGraph;    // output image

    // Get the desired image to draw into.
    if (!imageDst) {
        // Create an RGB image for graphing the data
        imageGraph = cvCreateImage(cvSize(w,h), 8, 3);

        // Clear the image
        cvSet(imageGraph, WHITE);
    }
    else {
        // Draw onto the given image.
        imageGraph = imageDst;
    }
    if (!imageGraph) {
        cerr << "ERROR in drawFloatGraph(): Couldn't create image of " << w << " x " << h << endl;
        exit(1);
    }
    CvScalar colorGraph = getGraphColor();    // use a different color each time.

    // If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
    if (fabs(minV) < 0.0000001f && fabs(maxV) < 0.0000001f) {
        for (int i=0; i<nArrayLength; i++) {
            float v = (float)arraySrc[i];
            if (v < minV)
                minV = v;
            if (v > maxV)
                maxV = v;
        }
    }
    float diffV = maxV - minV;
    if (diffV == 0)
        diffV = 0.00000001f;    // Stop a divide-by-zero error
    float fscale = (float)s / diffV;

    // Draw the horizontal & vertical axis
    int y0 = cvRound(minV*fscale);
    cvLine(imageGraph, cvPoint(b,h-(b-y0)), cvPoint(w-b, h-(b-y0)), BLACK);
    cvLine(imageGraph, cvPoint(b,h-(b)), cvPoint(b, h-(b+s)), BLACK);

    // Write the scale of the y axis
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.55,0.7, 0,1,CV_AA);    // For OpenCV 1.1
    if (showScale) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.5,0.6, 0,1, CV_AA);    // For OpenCV 2.0
        CvScalar clr = GREY;
        char text[16];
        snprintf(text, sizeof(text)-1, "%.1f", maxV);
        cvPutText(imageGraph, text, cvPoint(1, b+4), &font, clr);
        // Write the scale of the x axis
        snprintf(text, sizeof(text)-1, "%d", (nArrayLength-1) );
        cvPutText(imageGraph, text, cvPoint(w-b+4-5*strlen(text), (h/2)+10), &font, clr);
    }

    // Draw the values
    CvPoint ptPrev = cvPoint(b,h-(b-y0));    // Start the lines at the 1st point.
    for (int i=0; i<nArrayLength; i++) {
        int y = cvRound((arraySrc[i] - minV) * fscale);    // Get the values at a bigger scale
        int x = cvRound(i * xscale);
        CvPoint ptNew = cvPoint(b+x, h-(b+y));
        cvLine(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);    // Draw a line from the previous point to the new point
        ptPrev = ptNew;
    }

    // Write the graph label, if desired
    if (graphLabel != NULL && strlen(graphLabel) > 0) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
        cvPutText(imageGraph, graphLabel, cvPoint(30, 10), &font, CV_RGB(0,0,0));    // black text
    }

    return imageGraph;
}

// Draw the graph of an array of ints into imageDst or a new image, between minV & maxV if given.
// Remember to free the newly created image if imageDst is not given.
IplImage* drawIntGraph(const int *arraySrc, int nArrayLength, IplImage *imageDst, int minV, int maxV, int width, int height, char *graphLabel, bool showScale)
{
    int w = width;
    int h = height;
    int b = 10;        // border around graph within the image
    if (w <= 20)
        w = nArrayLength + b*2;    // width of the image
    if (h <= 20)
        h = 220;

    int s = h - b*2;// size of graph height
    float xscale = 1.0;
    if (nArrayLength > 1)
        xscale = (w - b*2) / (float)(nArrayLength-1);    // horizontal scale
    IplImage *imageGraph;    // output image

    // Get the desired image to draw into.
    if (!imageDst) {
        // Create an RGB image for graphing the data
        imageGraph = cvCreateImage(cvSize(w,h), 8, 3);

        // Clear the image
        cvSet(imageGraph, WHITE);
    }
    else {
        // Draw onto the given image.
        imageGraph = imageDst;
    }
    if (!imageGraph) {
        cerr << "ERROR in drawIntGraph(): Couldn't create image of " << w << " x " << h << endl;
        exit(1);
    }
    CvScalar colorGraph = getGraphColor();    // use a different color each time.

    // If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
    if (minV == 0 && maxV == 0) {
        for (int i=0; i<nArrayLength; i++) {
            int v = arraySrc[i];
            if (v < minV)
                minV = v;
            if (v > maxV)
                maxV = v;
        }
    }
    int diffV = maxV - minV;
    if (diffV == 0)
        diffV = 1;    // Stop a divide-by-zero error
    float fscale = (float)s / (float)diffV;

    // Draw the horizontal & vertical axis
    int y0 = cvRound(minV*fscale);
    cvLine(imageGraph, cvPoint(b,h-(b-y0)), cvPoint(w-b, h-(b-y0)), BLACK);
    cvLine(imageGraph, cvPoint(b,h-(b)), cvPoint(b, h-(b+s)), BLACK);

    // Write the scale of the y axis
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.55,0.7, 0,1,CV_AA);    // For OpenCV 1.1
    if (showScale) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.5,0.6, 0,1, CV_AA);    // For OpenCV 2.0
        CvScalar clr = GREY;
        char text[16];
        snprintf(text, sizeof(text)-1, "%d", maxV);
        cvPutText(imageGraph, text, cvPoint(1, b+4), &font, clr);
        // Write the scale of the x axis
        snprintf(text, sizeof(text)-1, "%d", (nArrayLength-1) );
        cvPutText(imageGraph, text, cvPoint(w-b+4-5*strlen(text), (h/2)+10), &font, clr);
    }

    // Draw the values
    CvPoint ptPrev = cvPoint(b,h-(b-y0));    // Start the lines at the 1st point.
    for (int i=0; i<nArrayLength; i++) {
        int y = cvRound((arraySrc[i] - minV) * fscale);    // Get the values at a bigger scale
        int x = cvRound(i * xscale);
        CvPoint ptNew = cvPoint(b+x, h-(b+y));
        cvLine(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);    // Draw a line from the previous point to the new point
        ptPrev = ptNew;
    }

    // Write the graph label, if desired
    if (graphLabel != NULL && strlen(graphLabel) > 0) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
        cvPutText(imageGraph, graphLabel, cvPoint(30, 10), &font, CV_RGB(0,0,0));    // black text
    }

    return imageGraph;
}

// Draw the graph of an array of uchars into imageDst or a new image, between minV & maxV if given..
// Remember to free the newly created image if imageDst is not given.
IplImage* drawUCharGraph(const uchar *arraySrc, int nArrayLength, IplImage *imageDst, int minV, int maxV, int width, int height, char *graphLabel, bool showScale)
{
    int w = width;
    int h = height;
    int b = 10;        // border around graph within the image
    if (w <= 20)
        w = nArrayLength + b*2;    // width of the image
    if (h <= 20)
        h = 220;

    int s = h - b*2;// size of graph height
    float xscale = 1.0;
    if (nArrayLength > 1)
        xscale = (w - b*2) / (float)(nArrayLength-1);    // horizontal scale
    IplImage *imageGraph;    // output image

    // Get the desired image to draw into.
    if (!imageDst) {
        // Create an RGB image for graphing the data
        imageGraph = cvCreateImage(cvSize(w,h), 8, 3);

        // Clear the image
        cvSet(imageGraph, WHITE);
    }
    else {
        // Draw onto the given image.
        imageGraph = imageDst;
    }
    if (!imageGraph) {
        cerr << "ERROR in drawUCharGraph(): Couldn't create image of " << w << " x " << h << endl;
        exit(1);
    }
    CvScalar colorGraph = getGraphColor();    // use a different color each time.

    // If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
    if (minV == 0 && maxV == 0) {
        for (int i=0; i<nArrayLength; i++) {
            int v = arraySrc[i];
            if (v < minV)
                minV = v;
            if (v > maxV)
                maxV = v;
        }
    }
    int diffV = maxV - minV;
    if (diffV == 0)
        diffV = 1;    // Stop a divide-by-zero error
    float fscale = (float)s / (float)diffV;

    // Draw the horizontal & vertical axis
    int y0 = cvRound(minV*fscale);
    cvLine(imageGraph, cvPoint(b,h-(b-y0)), cvPoint(w-b, h-(b-y0)), BLACK);
    cvLine(imageGraph, cvPoint(b,h-(b)), cvPoint(b, h-(b+s)), BLACK);

    // Write the scale of the y axis
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.55,0.7, 0,1,CV_AA);    // For OpenCV 1.1
    if (showScale) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.5,0.6, 0,1, CV_AA);    // For OpenCV 2.0
        CvScalar clr = GREY;
        char text[16];
        snprintf(text, sizeof(text)-1, "%d", maxV);
        cvPutText(imageGraph, text, cvPoint(1, b+4), &font, clr);
        // Write the scale of the x axis
        snprintf(text, sizeof(text)-1, "%d", (nArrayLength-1) );
        cvPutText(imageGraph, text, cvPoint(w-b+4-5*strlen(text), (h/2)+10), &font, clr);
    }

    // Draw the values
    CvPoint ptPrev = cvPoint(b,h-(b-y0));    // Start the lines at the 1st point.
    for (int i=0; i<nArrayLength; i++) {
        int y = cvRound((arraySrc[i] - minV) * fscale);    // Get the values at a bigger scale
        int x = cvRound(i * xscale);
        CvPoint ptNew = cvPoint(b+x, h-(b+y));
        cvLine(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);    // Draw a line from the previous point to the new point
        ptPrev = ptNew;
    }

    // Write the graph label, if desired
    if (graphLabel != NULL && strlen(graphLabel) > 0) {
        //cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
        cvPutText(imageGraph, graphLabel, cvPoint(30, 10), &font, CV_RGB(0,0,0));    // black text
    }

    return imageGraph;
}

// Display a graph of the given float array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawFloatGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showFloatGraph(const char *name, const float *arraySrc, int nArrayLength, int delay_ms, IplImage *background)
{
#ifdef USE_HIGHGUI
    // Draw the graph
    IplImage *imageGraph = drawFloatGraph(arraySrc, nArrayLength, background);

    // Display the graph into a window
    cvNamedWindow( name );
    cvShowImage( name, imageGraph );

    cvWaitKey( 10 );        // Note that cvWaitKey() is required for the OpenCV window to show!
    cvWaitKey( delay_ms );    // Wait longer to make sure the user has seen the graph

    cvReleaseImage(&imageGraph);
#endif
}

// Display a graph of the given int array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawIntGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showIntGraph(const char *name, const int *arraySrc, int nArrayLength, int delay_ms, IplImage *background)
{
#ifdef USE_HIGHGUI
    // Draw the graph
    IplImage *imageGraph = drawIntGraph(arraySrc, nArrayLength, background);

    // Display the graph into a window
    cvNamedWindow( name );
    cvShowImage( name, imageGraph );

    cvWaitKey( 10 );        // Note that cvWaitKey() is required for the OpenCV window to show!
    cvWaitKey( delay_ms );    // Wait longer to make sure the user has seen the graph

    cvReleaseImage(&imageGraph);
#endif
}

// Display a graph of the given unsigned char array.
// If background is provided, it will be drawn into, for combining multiple graphs using drawUCharGraph().
// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
void showUCharGraph(const char *name, const uchar *arraySrc, int nArrayLength, int delay_ms, IplImage *background)
{
#ifdef USE_HIGHGUI
    // Draw the graph
    IplImage *imageGraph = drawUCharGraph(arraySrc, nArrayLength, background);

    // Display the graph into a window
    cvNamedWindow( name );
    cvShowImage( name, imageGraph );

    cvWaitKey( 10 );        // Note that cvWaitKey() is required for the OpenCV window to show!
    cvWaitKey( delay_ms );    // Wait longer to make sure the user has seen the graph

    cvReleaseImage(&imageGraph);
#endif
}

// Simple helper function to easily view an image, with an optional pause.
void showImage(const IplImage *img, int delay_ms, char *name)
{
/*
#ifdef USE_HIGHGUI
    if (!name)
        name = "Image";
    cvNamedWindow(name, CV_WINDOW_AUTOSIZE);
    cvShowImage(name, img);
    cvWaitKey(delay_ms);
#endif
*/
}

//------------------------------------------------------------------------------
// Color conversion functions
//------------------------------------------------------------------------------

// Return a new image that is always greyscale, whether the input image was RGB or Greyscale.
// Remember to free the returned image using cvReleaseImage() when finished.
IplImage* convertImageToGreyscale(const IplImage *imageSrc)
{
    IplImage *imageGrey;
    // Either convert the image to greyscale, or make a copy of the existing greyscale image.
    // This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
    if (imageSrc->nChannels == 3) {
        imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
        cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
    }
    else {
        imageGrey = cvCloneImage(imageSrc);
    }
    return imageGrey;
}

// Do the color conversion of a single pixel, from RGB to HSV using Hue values between 0 to 255, whereas OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
inline void convertPixelRGBtoHSV_256(int bR, int bG, int bB, int &bH, int &bS, int &bV)
{
    float fR, fG, fB;
    float fH, fS, fV;
    const float FLOAT_TO_BYTE = 255.0f;
    const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

    // Convert from 8-bit integers to floats
    fR = bR * BYTE_TO_FLOAT;
    fG = bG * BYTE_TO_FLOAT;
    fB = bB * BYTE_TO_FLOAT;

    // Convert from RGB to HSV, using float ranges 0.0 to 1.0
    float fDelta;
    float fMin, fMax;
    int iMax;
    // Get the min & max, but use integer comparisons for slight speedup
    if (bB < bG) {
        if (bB < bR) {
            fMin = fB;
            if (bR > bG) {
                iMax = bR;
                fMax = fR;
            }
            else {
                iMax = bG;
                fMax = fG;
            }
        }
        else {
            fMin = fR;
            fMax = fG;
            iMax = bG;
        }
    }
    else {
        if (bG < bR) {
            fMin = fG;
            if (bB > bR) {
                fMax = fB;
                iMax = bB;
            }
            else {
                fMax = fR;
                iMax = bR;
            }
        }
        else {
            fMin = fR;
            fMax = fB;
            iMax = bB;
        }
    }
    fDelta = fMax - fMin;
    fV = fMax;                    // Value (Brightness).
    if (iMax != 0) {            // Make sure its not pure black.
        fS = fDelta / fMax;        // Saturation.
        float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);    // Make the Hues between 0.0 to 1.0 instead of 6.0
        if (iMax == bR) {        // between yellow & magenta.
            fH = (fG - fB) * ANGLE_TO_UNIT;
        }
        else if (iMax == bG) {    // between cyan & yellow.
            fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
        }
        else {                    // between magenta & cyan.
            fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
        }
        // Wrap outlier Hues around the circle.
        if (fH < 0.0f)
            fH += 1.0f;
        if (fH >= 1.0f)
            fH -= 1.0f;
    }
    else {
        // color is pure Black.
        fS = 0;
        fH = 0;    // undefined hue
    }

    // Convert from floats to 8-bit integers
    bH = (int)(0.5f + fH * 255.0f);
    bS = (int)(0.5f + fS * 255.0f);
    bV = (int)(0.5f + fV * 255.0f);

    // Clip the values to make sure it fits within the 8bits
    //if (bH > 255 || bH < 0 || bS > 255 || bS < 0 || bV > 255 || bV < 0) {
    //    cout << "Warning: HSV pixel(" << x << "," << y << ") is being clipped. " << bH << "," << bS << "," << bV << endl;
    //}
    if (bH > 255)
        bH = 255;
    if (bH < 0)
        bH = 0;
    if (bS > 255)
        bS = 255;
    if (bS < 0)
        bS = 0;
    if (bV > 255)
        bV = 255;
    if (bV < 0)
        bV = 0;
}

// Create a HSV image from the RGB image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated HSV image.
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB)
{
    // Create a blank HSV image
    IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
    if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
        LOG("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
        exit(1);
    }

    int h = imageRGB->height;                // Pixel height
    int w = imageRGB->width;                // Pixel width
    int rowSizeRGB = imageRGB->widthStep;    // Size of row in bytes, including extra padding
    char *imRGB = imageRGB->imageData;        // Pointer to the start of the image pixels.
    int rowSizeHSV = imageHSV->widthStep;    // Size of row in bytes, including extra padding
    char *imHSV = imageHSV->imageData;        // Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            int bB = *(uchar*)(pRGB+0);    // Blue component
            int bG = *(uchar*)(pRGB+1);    // Green component
            int bR = *(uchar*)(pRGB+2);    // Red component

            // Do the conversion.
            int bH, bS, bV;
            convertPixelRGBtoHSV_256(bR,bG,bB, bH,bS,bV);

            // Set the HSV pixel components
            uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
            *(pHSV+0) = bH;        // H component
            *(pHSV+1) = bS;        // S component
            *(pHSV+2) = bV;        // V component
        }
    }
    return imageHSV;
}

// Do the color conversion of a single pixel, from HSV to RGB using Hue values between 0 to 255, whereas OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
inline void convertPixelHSVtoRGB_256(int bH, int bS, int bV, int &bR, int &bG, int &bB)
{
    float fH, fS, fV;
    float fR, fG, fB;
    const float FLOAT_TO_BYTE = 255.0f;
    const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

    // Convert from 8-bit integers to floats
    fH = (float)bH * BYTE_TO_FLOAT;
    fS = (float)bS * BYTE_TO_FLOAT;
    fV = (float)bV * BYTE_TO_FLOAT;

    // Convert from HSV to RGB, using float ranges 0.0 to 1.0
    int iI;
    float fI, fF, p, q, t;

    if( bS == 0 ) {
        // achromatic (grey)
        fR = fG = fB = fV;
    }
    else {
        //if (bH < 0 || bH >= 255 || bS < 0 || bS > 255 || bV < 0 || bV > 255) {
        //    cout << "ERROR: HSVi pixel(" << x << "," << y << ") is being clipped. " << bH << "," << bS << "," << bV << endl;
        //    cout << "ERROR: HSVf pixel(" << x << "," << y << ") is being clipped. " << fH << "," << fS << "," << fV << endl;
        //}

        // If Hue == 1.0, then wrap it around the circle to 0.0
        if (fH >= 1.0f)
            fH = 0.0f;

        fH *= 6.0;            // sector 0 to 5
        fI = floor( fH );        // integer part of h (0,1,2,3,4,5 or 6)
        iI = (int) fH;            //        "        "        "        "
        fF = fH - fI;            // factorial part of h (0 to 1)

        p = fV * ( 1.0f - fS );
        q = fV * ( 1.0f - fS * fF );
        t = fV * ( 1.0f - fS * ( 1.0f - fF ) );

        switch( iI ) {
            case 0:
                fR = fV;
                fG = t;
                fB = p;
                break;
            case 1:
                fR = q;
                fG = fV;
                fB = p;
                break;
            case 2:
                fR = p;
                fG = fV;
                fB = t;
                break;
            case 3:
                fR = p;
                fG = q;
                fB = fV;
                break;
            case 4:
                fR = t;
                fG = p;
                fB = fV;
                break;
            default:        // case 5 (or 6):
                fR = fV;
                fG = p;
                fB = q;
                break;
        }
    }

    // Convert from floats to 8-bit integers
    bR = (int)(fR * FLOAT_TO_BYTE);
    bG = (int)(fG * FLOAT_TO_BYTE);
    bB = (int)(fB * FLOAT_TO_BYTE);

    // Clip the values to make sure it fits within the 8bits
    //if (bR > 255 || bR < 0 || bG > 255 || bG < 0 || bB > 255 || bB < 0) {
    //    cout << "Warning: RGB pixel(" << x << "," << y << ") is being clipped. " << bR << "," << bG << "," << bB << endl;
    //}
    if (bR > 255)
        bR = 255;
    if (bR < 0)
        bR = 0;
    if (bG > 255)
        bG = 255;
    if (bG < 0)
        bG = 0;
    if (bB > 255)
        bB = 255;
    if (bB < 0)
        bB = 0;
}

// Create an RGB image from the HSV image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated RGB image.
IplImage* convertImageHSVtoRGB(const IplImage *imageHSV)
{
    // Create a blank RGB image
    IplImage *imageRGB = cvCreateImage(cvGetSize(imageHSV), 8, 3);
    if (!imageRGB || imageHSV->depth != 8 || imageHSV->nChannels != 3) {
        LOG("ERROR in convertImageHSVtoRGB()! Bad input image.\n");
        exit(1);
    }

    int h = imageHSV->height;                // Pixel height
    int w = imageHSV->width;                // Pixel width
    int rowSizeHSV = imageHSV->widthStep;    // Size of row in bytes, including extra padding
    char *imHSV = imageHSV->imageData;        // Pointer to the start of the image pixels.
    int rowSizeRGB = imageRGB->widthStep;    // Size of row in bytes, including extra padding
    char *imRGB = imageRGB->imageData;        // Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the HSV pixel components
            uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
            int bH = *(uchar*)(pHSV+0);    // H component
            int bS = *(uchar*)(pHSV+1);    // S component
            int bV = *(uchar*)(pHSV+2);    // V component

            // Do the conversion.
            int bB, bG, bR;
            convertPixelHSVtoRGB_256(bH,bS,bV,  bR,bG,bB);

            // Set the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            *(pRGB+0) = bB;        // B component
            *(pRGB+1) = bG;        // G component
            *(pRGB+2) = bR;        // R component
        }
    }
    return imageRGB;
}

// Do the color conversion of a single pixel, from RGB to HSV using Hue values between 0 to 179 just like OpenCV does, instead of Hues upto 255.
void convertPixelRGBtoHSV_180(int bR, int bG, int bB, int &bH, int &bS, int &bV)
{
    // Do the conversion with Hues between 0 to 255.
    convertPixelRGBtoHSV_256(bR,bG,bB, bH,bS,bV);

    // Convert to a Hue of 180 range from 256 range.
    bH = 179 * bH / 255;
}

// Do the color conversion of a single pixel, from HSV to RGB using Hue values between 0 to 179 just like OpenCV does, instead of Hues upto 255.
void convertPixelHSVtoRGB_180(int bH, int bS, int bV, int &bR, int &bG, int &bB)
{
    // Convert from a Hue of 180 range to 256 range.
    bH = 255 * bH / 179;

    // Do the conversion with Hues between 0 to 255.
    convertPixelHSVtoRGB_256(bH,bS,bV,  bR,bG,bB);
}

// Create an RGB image from the YIQ image using an approximation of NTSC conversion(ref: "YIQ" Wikipedia page).
// Remember to free the generated RGB image.
IplImage* convertImageYIQtoRGB(const IplImage *imageYIQ)
{
    float fY, fI, fQ;
    float fR, fG, fB;
    const float FLOAT_TO_BYTE = 255.0f;
    //const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;
    const float MIN_I = -0.5957f;
    const float MIN_Q = -0.5226f;
    const float Y_TO_FLOAT = 1.0f / 255.0f;
    const float I_TO_FLOAT = -2.0f * MIN_I / 255.0f;
    const float Q_TO_FLOAT = -2.0f * MIN_Q / 255.0f;

    // Create a blank RGB image
    IplImage *imageRGB = cvCreateImage(cvGetSize(imageYIQ), 8, 3);
    if (!imageRGB || imageYIQ->depth != 8 || imageYIQ->nChannels != 3) {
        LOG("ERROR in convertImageYIQtoRGB()! Bad input image.\n");
        exit(1);
    }

    int h = imageYIQ->height;                // Pixel height
    int w = imageYIQ->width;                // Pixel width
    int rowSizeYIQ = imageYIQ->widthStep;    // Size of row in bytes, including extra padding
    char *imYIQ = imageYIQ->imageData;        // Pointer to the start of the image pixels.
    int rowSizeRGB = imageRGB->widthStep;    // Size of row in bytes, including extra padding
    char *imRGB = imageRGB->imageData;        // Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the YIQ pixel components
            uchar *pYIQ = (uchar*)(imYIQ + y*rowSizeYIQ + x*3);
            int bY = *(uchar*)(pYIQ+0);    // Y component
            int bI = *(uchar*)(pYIQ+1);    // I component
            int bQ = *(uchar*)(pYIQ+2);    // Q component

            // Convert from 8-bit integers to floats
            fY = (float)bY * Y_TO_FLOAT;
            fI = (float)bI * I_TO_FLOAT + MIN_I;
            fQ = (float)bQ * Q_TO_FLOAT + MIN_Q;
            // Convert from YIQ to RGB
            // where R,G,B are 0-1, Y is 0-1, I is -0.5957 to +0.5957, Q is -0.5226 to +0.5226.
            fR =  fY  + 0.9563f * fI + 0.6210f * fQ;
            fG =  fY  - 0.2721f * fI - 0.6474f * fQ;
            fB =  fY  - 1.1070f * fI + 1.7046f * fQ;
            // Convert from floats to 8-bit integers
            int bR = (int)(fR * FLOAT_TO_BYTE);
            int bG = (int)(fG * FLOAT_TO_BYTE);
            int bB = (int)(fB * FLOAT_TO_BYTE);

            // Clip the values to make sure it fits within the 8bits
            //if (bR > 255 || bR < 0 || bG > 255 || bG < 0 || bB > 255 || bB < 0) {
            //    cout << "Warning: RGB pixel(" << x << "," << y << ") is being clipped. " << bR << "," << bG << "," << bB << endl;
            //}
            if (bR > 255)
                bR = 255;
            if (bR < 0)
                bR = 0;
            if (bG > 255)
                bG = 255;
            if (bG < 0)
                bG = 0;
            if (bB > 255)
                bB = 255;
            if (bB < 0)
                bB = 0;

            // Set the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            *(pRGB+0) = bB;        // B component
            *(pRGB+1) = bG;        // G component
            *(pRGB+2) = bR;        // R component
        }
    }
    return imageRGB;
}

//------------------------------------------------------------------------------
// 2D Point functions
//------------------------------------------------------------------------------
CvPoint2D32f addPointF(const CvPoint2D32f pointA, const CvPoint2D32f pointB)
{
    CvPoint2D32f ret;
    ret.x = pointA.x + pointB.x;
    ret.y = pointA.y + pointB.y;
    return ret;
}

CvPoint2D32f subtractPointF(const CvPoint2D32f pointA, const CvPoint2D32f pointB)
{
    CvPoint2D32f ret;
    ret.x = pointA.x - pointB.x;
    ret.y = pointA.y - pointB.y;
    return ret;
}

CvPoint2D32f scalePointF(const CvPoint2D32f point, float scale)
{
    CvPoint2D32f ret;
    ret.x = point.x * scale;
    ret.y = point.y * scale;
    return ret;
}

CvPoint2D32f rotatePointF(const CvPoint2D32f point, float angleDegrees)
{
    CvPoint2D32f ret;
    float dx = point.x;
    float dy = point.y;
    float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
    float cosA = cos(angleRadians);
    float sinA = sin(angleRadians);
    ret.x = (dx * cosA - dy * sinA);
    ret.y = (dx * sinA + dy * cosA);
    return ret;
}

CvPoint2D32f rotatePointAroundPointF(const CvPoint2D32f point, const CvPoint2D32f origin, float angleDegrees)
{
    CvPoint2D32f ret;
    float dx = point.x - origin.x;
    float dy = point.y - origin.y;
    float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
    float cosA = cos(angleRadians);
    float sinA = sin(angleRadians);
    ret.x = (dx * cosA - dy * sinA) + origin.x;
    ret.y = (dx * sinA + dy * cosA) + origin.y;
    return ret;
}

CvPoint2D32f scalePointAroundPointF(const CvPoint2D32f point, const CvPoint2D32f origin, float scale)
{
    CvPoint2D32f ret;
    float dx = point.x - origin.x;
    float dy = point.y - origin.y;
    ret.x = (dx * scale) + origin.x;
    ret.y = (dy * scale) + origin.y;
    return ret;
}

// Return (p * s), to a maximum value of maxVal
float scaleValueF(float p, float s, float maxVal)
{
    float r = p * s;
    if (r > maxVal)
    {
        r = maxVal;
    }
    return r;
}

// Return (p * s), to a maximum value of maxVal
int scaleValueI(int p, float s, int maxVal)
{
    int r = cvRound( ((float)p) * s );
    if (r > maxVal)
    {
        r = maxVal;
    }
    return r;
}


// Calculate the distance between the 2 given points.
float findDistanceBetweenPointsF(const CvPoint2D32f p1, const CvPoint2D32f p2)
{
    // Calc the pythagoras distance.
    float dx = (p1.x - p2.x);
    float dy = (p1.y - p2.y);
    return sqrt((float) (dx * dx + dy * dy));
}
float findDistanceBetweenPointsI(const CvPoint p1, const CvPoint p2)
{
    // Calc the pythagoras distance.
    int dx = (p1.x - p2.x);
    int dy = (p1.y - p2.y);
    return sqrt((float) (dx * dx + dy * dy));
}

// Calculate the angle between the 2 given floating-point points (in degrees).
float findAngleBetweenPointsF(const CvPoint2D32f p1, const CvPoint2D32f p2)
{
    // Calculate the angle of in-plane-rotation of the face based on the angle of the 2 eye positions.
    float dx = (float)(p2.x - p1.x);
    if (dx == 0.0f)
        dx = 0.00000001f;    // Stop a divide-by-zero error.
    float radians = atan2((float) (p2.y - p1.y), dx);    // angle = inv_tan(dy / dx).
    return (radians * 180.0f / (float)CV_PI);    // convert to degrees from radians.
}
// Calculate the angle between the 2 given integer points (in degrees).
float findAngleBetweenPointsI(const CvPoint p1, const CvPoint p2)
{
    // Calculate the angle of in-plane-rotation of the face based on the angle of the 2 eye positions.
    float dx = (float)(p2.x - p1.x);
    if (dx == 0.0f)
        dx = 0.00000001f;    // Stop a divide-by-zero error.
    float radians = atan2((float) (p2.y - p1.y), dx);    // angle = inv_tan(dy / dx).
    return (radians * 180.0f / (float)CV_PI);    // convert to degrees from radians.
}

// Print the label and then the rect to the console for easy debugging
void printPoint(const CvPoint pt, const char *label)
{
    if (label)
        std::cout << label << ": ";
    std::cout << "[Point] at (" << pt.x << "," << pt.y << ")" << std::endl;
}

// Print the label and then the rect to the console for easy debugging
void printPointF(const CvPoint2D32f pt, const char *label)
{
    if (label)
        std::cout << label << ": ";
    std::cout << "[Point32f] at (" << pt.x << "," << pt.y << ")" << std::endl;
}


//------------------------------------------------------------------------------
// Rectangle region functions
//------------------------------------------------------------------------------

// Enlarge or shrink the rectangle size & center by a given scale.
// If w & h are given, will make sure the rectangle stays within the bounds if it is enlarged too much.
// Note that for images, w should be (width-1) and h should be (height-1).
CvRect scaleRect(const CvRect rectIn, float scaleX, float scaleY, int w, int h)
{
    CvRect rect;
    // Scale the image region position & size
    rect.x = cvRound(scaleX * (float)rectIn.x);
    rect.y = cvRound(scaleY * (float)rectIn.y);
    rect.width = cvRound(scaleX * (float)rectIn.width);
    rect.height = cvRound(scaleY * (float)rectIn.height);
    // Make sure it doesn't go outside the image
    if (w > 0 && h > 0) {
        if (rect.x + rect.width > w)
            rect.width = w - rect.x;
        if (rect.y + rect.height > h)
            rect.height = h - rect.y;
    }
    return rect;
}

// Enlarge or shrink the rectangle region by a given scale without moving its center, and possibly add a border around it.
// If w & h are given, will make sure the rectangle stays within the bounds if it is enlarged too much.
// Note that for images, w should be (width-1) and h should be (height-1).
CvRect scaleRectInPlace(const CvRect rectIn, float scaleX, float scaleY, float borderX, float borderY, int w, int h)
{
    CvRect rect = rectIn;    // Make a local copy
    // Scale the image region size and add an extra border to the region size
    rect.x -= cvRound((scaleX - 1.0f) * (float)rect.width * 0.5f + borderX);
    rect.y -= cvRound((scaleY - 1.0f) * (float)rect.height * 0.5f + borderY);
    rect.width = cvRound(scaleX * (float)rect.width + 2.0f * borderX);
    rect.height = cvRound(scaleY * (float)rect.height + 2.0f * borderY);
    // Make sure it doesn't go outside the image
    if (w > 0 && h > 0) {
        if (rect.x < 0) {
            rect.width += rect.x;    // shrink the width, since 'x' is negative
            rect.x = 0;
        }
        if (rect.x + rect.width > w)
            rect.width = w - rect.x;
        if (rect.y < 0) {
            rect.height += rect.y;    // shrink the height, since 'y' is negative
            rect.y = 0;
        }
        if (rect.y + rect.height > h)
            rect.height = h - rect.y;
    }
    return rect;
}

// Return a new rect that the same as rectA when shifted by (rectB.x, rectB.y).
CvRect offsetRect(const CvRect rectA, const CvRect rectB)
{
    CvRect rectC;
    rectC.x = rectA.x + rectB.x;
    rectC.y = rectA.y + rectB.y;
    rectC.width = rectA.width;
    rectC.height = rectA.height;
    return rectC;
}

// Return a new rect that the same as rectA when shifted by (pt.x, pt.y).
CvRect offsetRectPt(const CvRect rectA, const CvPoint pt)
{
    CvRect rectC;
    rectC.x = rectA.x + pt.x;
    rectC.y = rectA.y + pt.y;
    rectC.width = rectA.width;
    rectC.height = rectA.height;
    return rectC;
}

// Draw a rectangle around the given object.
void drawRect(IplImage *img, const CvRect rect, const CvScalar color)
{
    CvPoint p1, p2;
    p1.x = rect.x;
    p1.y = rect.y;
    p2.x = min(rect.x + rect.width-1, img->width-1);    // Make sure the end-point is within the image
    p2.y = min(rect.y + rect.height-1, img->height-1);    //        "        "        "
    cvRectangle(img, p1, p2, color, 1);
}
// Draw a filled rectangle around the given object.
void drawRectFilled(IplImage *img, const CvRect rect, const CvScalar color)
{
    CvPoint p1, p2;
    p1.x = rect.x;
    p1.y = rect.y;
    p2.x = min(rect.x + rect.width-1, img->width-1);    // Make sure the end-point is within the image
    p2.y = min(rect.y + rect.height-1, img->height-1);    //        "        "        "
    cvRectangle(img, p1, p2, color, CV_FILLED);
}

// Draw a crossbar at the given position.
void drawCross(IplImage *img, const CvPoint pt, int radius, const CvScalar color)
{
    CvPoint p1, p2, p3, p4;
    // Make sure the points are within the image
    p1 = cvPoint( max(pt.x-radius,0),             pt.y );
    p2 = cvPoint( min(pt.x+radius,img->width-1),  pt.y );
    p3 = cvPoint( pt.x,                           max(pt.y-radius,0) );
    p4 = cvPoint( pt.x,                           min(pt.y+radius,img->height-1) );
    // Draw a horizontal line through the given point
    cvLine(img, p1, p2, color);
    // Draw a vertical line through the given point
    cvLine(img, p3, p4, color);
}

// Print the label and then the rect to the console for easy debugging
void printRect(const CvRect rect, const char *label)
{
    if (label)
        std::cout << label << ": ";
    std::cout << "[Rect] at (" << rect.x << "," << rect.y << ") of size " << rect.width << "x" << rect.height <<
        ", where bottom-right corner is at (" << (rect.x + rect.width-1) << "," << (rect.y + rect.height-1) << ")" << std::endl;
}

// Make sure the given rectangle is completely within the given image dimensions.
CvRect cropRect(const CvRect rectIn, int w, int h)
{
    CvRect roi = CvRect(rectIn);
    // Make sure the displayed image is within the viewing dimensions
    if (roi.x + roi.width > w)    // Limit the bottom-right from past the image
        roi.width = w - roi.x;
    if (roi.y + roi.height > h)
        roi.height = h - roi.y;
    if (roi.x < 0)                // Limit the top-left from before the image
        roi.x = 0;
    if (roi.y < 0)
        roi.y = 0;
    if (roi.x > w-1)            // Limit the top-left from after the image
        roi.x = w-1;
    if (roi.y > h-1)
        roi.y = h-1;
    if (roi.width < 0)            // Limit the negative sizes
        roi.width = 0;
    if (roi.height < 0)
        roi.height = 0;
    if (roi.width > w)            // Limit the large sizes
        roi.width = w - roi.x;
    if (roi.height > h)
        roi.height = h - roi.y;
    return roi;
}


//------------------------------------------------------------------------------
// Image transforming functions
//------------------------------------------------------------------------------

/*
// Returns a new image that is a cropped version of the original image.
IplImage* slow_cropImage(const IplImage *img, const CvRect region)
{
    IplImage *imageTmp,*imageRGB;
    CvSize size;
    size.height = img->height;
    size.width = img-> width;

    if (img->width <= 0 || img->height <= 0 || region.width <= 0 || region.height <= 0) {
        std::cerr << "ERROR in cropImage(): dimensions of image or region are invalid." << endl;
        printRect(region, "crop region");
        printImageInfo(img, "crop image");
        exit(1);
    }

    if (img->depth != IPL_DEPTH_8U) {
        std::cerr << "ERROR in cropImage(): Unknown image depth of " << img->depth << " given instead of 8." << std::endl;
        exit(1);
    }

    // First create a new (color or greyscale) IPL Image and copy contents of img into it.
    imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
    cvCopy(img, imageTmp);

    // Create a new image of the detected region
    //printf("Cropping image at x = %d, y = %d...", faces[i].x, faces[i].y);
    //printf("Setting region of interest...");

    // Make sure the crop region is completely within the image region.
    //cropRegion = cropRect(cropRegion, size.width-1, size.height-1);    // Make sure it fits within (0,0) and (w,h)
    // Set the region of interest to that surrounding the face.
    cvSetImageROI(imageTmp, region);
    // Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
    size.width = region.width;
    size.height = region.height;
    imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
    cvCopy(imageTmp, imageRGB);    // Copy just the region.

    cvReleaseImage( &imageTmp );
    return imageRGB;
}
*/

// Returns a new image that is a cropped version (rectangular cut-out) of the original image.
IplImage* cropImage(const IplImage *img, const CvRect region)
{
    IplImage *imageCropped;
    CvSize size;

    if (img->width <= 0 || img->height <= 0 || region.width <= 0 || region.height <= 0) {
        LOG("ERROR in cropImage(): dimensions of image or region are invalid.");
        printRect(region, "crop region");
        printImageInfo(img, "crop image");
        exit(1);
    }

    // Set the desired region of interest.
    cvSetImageROI((IplImage*)img, region);
    // Copy region of interest into a new iplImage and return it
    size.width = region.width;
    size.height = region.height;
    imageCropped = cvCreateImage(size, img->depth, img->nChannels);
    cvCopy(img, imageCropped);    // Copy just the region.

    cvResetImageROI((IplImage*)img);
    return imageCropped;
}

// Creates a new image copy that is of a desired size. The aspect ratio will be kept constant if 'keepAspectRatio' is true,
// by cropping undesired parts so that only pixels of the original image are shown, instead of adding extra blank space.
// Remember to free the new image later.
IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight, bool keepAspectRatio)
{
    IplImage *outImg = 0;
    int origWidth = 1;
    int origHeight = 1;
    if (origImg) {
        origWidth = origImg->width;
        origHeight = origImg->height;
    }
    if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
        std::cerr << "ERROR: Bad desired image size of " << newWidth << "x" << newHeight << " in resizeImage().\n";
        if (origImg)
            std::cerr << "Image was " << origImg->width << "x" << origImg->height << endl;
        exit(1);
    }

    if (keepAspectRatio) {
        //cerr << "ResizeImage :" << origWidth << "x" << origHeight << " -> " << newWidth << "x" << newHeight << " (rOld" << (origWidth / (float)origHeight) << ", rNew" << (newWidth / (float)newHeight) << endl;
        // Resize the image without changing its aspect ratio, by cropping off the edges and enlarging the middle section.
        CvRect r;
        float origAspect = (origWidth / (float)origHeight);    // input aspect ratio
        float newAspect = (newWidth / (float)newHeight);    // output aspect ratio
        if (origAspect > newAspect) {    // crop width to be origHeight * newAspect
            int tw = (origHeight * newWidth) / newHeight;
            r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
        }
        else {    // crop height to be origWidth / newAspect
            int th = (origWidth * newHeight) / newWidth;
            r = cvRect(0, (origHeight - th)/2, origWidth, th);
        }
        //cerr << "cropping image to (" << r.width << "x" << r.height << " at (" << r.x << "," << r.y << ")." << endl;
        IplImage *croppedImg = cropImage(origImg, r);
        // Call this function again, but with the new aspect ratio image.
        //cerr << "calling resizeImage(" << newWidth << "," << newHeight << "false)" << endl;
        outImg = resizeImage(croppedImg, newWidth, newHeight, false);    // do a scaled image resize, since the aspect ratio is correct now.
        cvReleaseImage( &croppedImg );

    }
    else {

        // Scale the image to the new dimensions, even if the aspect ratio will be changed.
        outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
        if (newWidth > origImg->width && newHeight > origImg->height) {
            // Make the image larger
            //printf("Making the image larger\n"); fflush(stdout);
            cvResetImageROI((IplImage*)origImg);
            cvResize(origImg, outImg, CV_INTER_LINEAR);    // CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
        }
        else {
            // Make the image smaller
            //printf("Making the image smaller\n"); fflush(stdout);
            cvResetImageROI((IplImage*)origImg);
            cvResize(origImg, outImg, CV_INTER_AREA);    // CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
        }

    }
    return outImg;
}

// Rotate the image clockwise and possibly scale the image. Use 'mapRotatedImagePoint()' to map pixels from the src to dst image.
IplImage *rotateImage(const IplImage *src, float angleDegrees, float scale)
{
    // Create a map_matrix, where the left 2x2 matrix is the transform and the right 2x1 is the dimensions.
    float m[6];
    CvMat M = cvMat(2, 3, CV_32F, m);
    int w = src->width;
    int h = src->height;

    float divscale = 1.0f;
    if (scale != 1.0f && scale > 1e-20)
        divscale = 1.0f / scale;
    float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
    m[0] = (float)(cos(angleRadians) * divscale);
    m[1] = (float)(sin(angleRadians) * divscale);
    m[3] = -m[1];
    m[4] = m[0];
    m[2] = w*0.5f;
    m[5] = h*0.5f;

    // Make a spare image for the result
    CvSize sizeRotated;
    sizeRotated.width = cvRound(scale * w);
    sizeRotated.height = cvRound(scale * h);

    // Rotate and scale
    IplImage *imageRotated = cvCreateImage( sizeRotated, src->depth, src->nChannels );

    // Transform the image
    cvGetQuadrangleSubPix( src, imageRotated, &M);

    return imageRotated;
}

// Get the position of a pixel in the image after the rotateImage() operation.
CvPoint2D32f mapRotatedImagePoint(const CvPoint2D32f pointOrig, const IplImage *image, float angleDegrees, float scale)
{
    // Get the old image center.
    CvPoint2D32f ptImageCenterOrig = cvPoint2D32f(image->width / 2.0f, image->height / 2.0f);
    // Get the new image center (after the rotation & scale that was performed to the image by 'rotateImage()').
    CvPoint2D32f ptImageCenterNew = scalePointF(ptImageCenterOrig, scale);
    // Get the relative coords of the point
    CvPoint2D32f relPOrig = subtractPointF(pointOrig, ptImageCenterOrig);    // relative coords
    // Rotate & scale the relative coords of the point
    CvPoint2D32f relPNew = rotatePointF(relPOrig, angleDegrees);
    relPNew = scalePointF(relPNew, scale);
    // Get the absolute coords of the new point
    CvPoint2D32f ptPNew = addPointF(relPNew, ptImageCenterNew);
    return ptPNew;
}

//------------------------------------------------------------------------------
// Image utility functions
//------------------------------------------------------------------------------

// Do Bilateral Filtering to smooth the image noise but preserve the edges.
// A smoothness of 5 is very little filtering, and 100 is very high filtering.
// Remember to free the returned image.
IplImage* smoothImageBilateral(const IplImage *src, float smoothness)
{
    IplImage *imageSmooth = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
    IplImage *imageOut = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
    // Do bilateral fitering on the input image
    cvSmooth( src, imageSmooth, CV_BILATERAL, 5, 5, smoothness, smoothness );

    // Mix the smoothed image with the original image
    cvAddWeighted( src, 0.70, imageSmooth, 0.70, 0.0, imageOut );

    //saveImage("bilatA.png", src);
    //saveImage("bilatB.png", imageSmooth);
    //saveImage("bilatC.png", imageOut);
    cvReleaseImage(&imageSmooth);
    return imageOut;
}


// Paste multiple images next to each other as a single image, for saving or displaying.
// Remember to free the returned image.
// Sample usage: cvSaveImage("out.png", combineImages(2, img1, img2) );
// Modified by Shervin from the cvShowManyImages() function on the OpenCVWiki by Parameswaran.
// 'combineImagesResized()' will resize all images to 300x300, whereas 'combineImages()' doesn't resize the images at all.
IplImage* combineImagesResized(int nArgs, ...)
{
    // img - Used for getting the arguments
    IplImage *img;

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return NULL;
    }
    else if(nArgs > 12) {
        printf("Number of arguments too large....\n");
        return NULL;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    // Create a new 3 channel image
    DispImage = cvCreateImage( cvSize(100 + size*w, 60 + size*h), 8, 3 );

    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);

    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        // Get the Pointer to the IplImage
        img = va_arg(args, IplImage*);

        // Make sure a proper image has been obtained
        if(img) {

            // Find the width and height of the image
            x = img->width;
            y = img->height;

            // Find whether height or width is greater in order to resize the image
            max = (x > y)? x: y;

            // Find the scaling factor to resize the image
            scale = (float) ( (float) max / size );

            // Used to Align the images
            if( i % w == 0 && m!= 20) {
                m = 20;
                n+= 20 + size;
            }

            // Make sure we have a color image. If its greyscale, then convert it to color.
            IplImage *colorImg = 0;
            IplImage *currImg = img;
            if (img->nChannels == 1) {
                colorImg = cvCreateImage(cvSize(img->width, img->height), 8, 3 );
                //std::cout << "[Converting greyscale image " << greyImg->width << "x" << greyImg->height << "px to color for combineImages()]" << std::endl;
                cvCvtColor( img, colorImg, CV_GRAY2BGR );
                currImg = colorImg;    // Use the greyscale version as the input.
            }

            // Set the image ROI to display the current image
            cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

            // Resize the input image and copy it to the Single Big Image
            cvResize(currImg, DispImage, CV_INTER_CUBIC);

            // Reset the ROI in order to display the next image
            cvResetImageROI(DispImage);

            if (colorImg)
                cvReleaseImage(&colorImg);
        }
        else {    // This input image is NULL
            //printf("Error in combineImages(): Bad image%d given as argument\n", i);
            //cvReleaseImage(&DispImage);    // Release the image and return
            //return NULL;
        }
    }

    // End the number of arguments
    va_end(args);

    return DispImage;
}



// Paste multiple images next to each other as a single image, for saving or displaying.
// Remember to free the returned image.
// Sample usage: cvSaveImage("out.png", combineImages(2, img1, img2) );
// Modified by Shervin from the cvShowManyImages() function on the OpenCVWiki by Parameswaran.
// 'combineImagesResized()' will resize all images to 300x300, whereas 'combineImages()' doesn't resize the images at all.
IplImage* combineImages(int nArgs, ...)
{
    const int MAX_COMBINED_IMAGES = 6;
    int col1Width, col2Width;
    int row1Height, row2Height, row3Height;
    IplImage *imageArray[MAX_COMBINED_IMAGES];
    int xPos[MAX_COMBINED_IMAGES];
    int yPos[MAX_COMBINED_IMAGES];
    int wImg[MAX_COMBINED_IMAGES] = {0};    // image dimensions are assumed to be 0, if they dont exist.
    int hImg[MAX_COMBINED_IMAGES] = {0};
    //int rows, columns;        // number of rows & cols of images.
    int wP, hP;                // dimensions of the combined image.
    int i;
    int nGoodImages = 0;
    IplImage *combinedImage;
    int B = 5;        // Border size, in pixels

    // Load all the images that were passed as arguments
    va_list args;    // Used to get the arguments passed
    va_start(args, nArgs);
    for (i = 0; i < nArgs; i++)    {
        // Get the Pointer to the IplImage
        IplImage *img = va_arg(args, IplImage*);
        // Make sure a proper image has been obtained, and that there aren't too many images already.
        if ((img != 0 && img->width > 0 && img->height > 0) && (nGoodImages < MAX_COMBINED_IMAGES) ) {
            // Add the new image to the array of images
            imageArray[nGoodImages] = img;
            wImg[nGoodImages] = img->width;
            hImg[nGoodImages] = img->height;
            nGoodImages++;
        }
    }

    // If the number of arguments is lesser than 0 or greater than 12,
    // return without displaying
    if( nGoodImages <= 0 || nGoodImages > MAX_COMBINED_IMAGES ) {
        printf("Error in combineImages(): Cant display %d of %d images\n", nGoodImages, nArgs);
        return NULL;
    }

    // Determine the size of the combined image & number of rows/cols.
    //columns = MIN(nGoodImages, 2);    // 1 or 2 columns
    //rows = (nGoodImages-1) / 2;        // 1 or 2 or 3 or ... rows
    col1Width = MAX(wImg[0], MAX(wImg[2], wImg[4]));
    col2Width = MAX(wImg[1], MAX(wImg[3], wImg[5]));
    row1Height = MAX(hImg[0], hImg[1]);
    row2Height = MAX(hImg[2], hImg[3]);
    row3Height = MAX(hImg[4], hImg[5]);
    wP = B + col1Width + B + (col2Width ? col2Width + B : 0);
    hP = B + row1Height + B + (row2Height ? row2Height + B : 0) + (row3Height ? row3Height + B : 0);
    xPos[0] = B;
    yPos[0] = B;
    xPos[1] = B + col1Width + B;
    yPos[1] = B;
    xPos[2] = B;
    yPos[2] = B + row1Height + B;
    xPos[3] = B + col1Width + B;
    yPos[3] = B + row1Height + B;
    xPos[4] = B;
    yPos[4] = B + row1Height + B + row2Height + B;
    xPos[5] = B + col1Width + B;
    yPos[5] = B + row1Height + B + row2Height + B;

    // Create a new RGB image
    combinedImage = cvCreateImage( cvSize(wP, hP), 8, 3 );
    if (!combinedImage)
        return NULL;

    // Clear the background
    cvSet(combinedImage, CV_RGB(50,50,50));

    for (i=0; i < nGoodImages; i++) {
        IplImage *img = imageArray[i];

        // Make sure we have a color image. If its greyscale, then convert it to color.
        IplImage *colorImg = 0;
        if (img->nChannels == 1) {
            colorImg = cvCreateImage(cvSize(img->width, img->height), 8, 3 );
            cvCvtColor( img, colorImg, CV_GRAY2BGR );
            img = colorImg;    // Use the greyscale version as the input.
        }

        // Set the image ROI to display the current image
        cvSetImageROI(combinedImage, cvRect(xPos[i], yPos[i], img->width, img->height));
        // Draw this image into its position
        cvCopy(img, combinedImage);
        // Reset the ROI in order to display the next image
        cvResetImageROI(combinedImage);

        if (colorImg)
            cvReleaseImage(&colorImg);
    }

    // End the number of arguments
    va_end(args);

    return combinedImage;
}


// Blend color images 'image1' and 'image2' using an 8-bit alpha-blending mask channel.
// Equivalent to this operation on each pixel: imageOut = image1 * (1-(imageAlphaMask/255)) + image2 * (imageAlphaMask/255)
// So if a pixel in imageAlphMask is 0, then that pixel in imageOut will be image1, or if imageAlphaMask is 255 then imageOut is image2,
// or if imageAlphaMask was 200 then imageOut would be: image1 * 0.78 + image2 * 0.22
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* blendImage(const IplImage* image1, const IplImage* image2, const IplImage* imageAlphaMask)
{
    // Make sure that image1 & image2 are RGB UCHAR images, and imageAlphaMask is an 8-bit UCHAR image.
    if (!image1 || image1->width <= 0 || image1->height <= 0 || image1->depth != 8 || image1->nChannels != 3) {
        std::cout << "Error in blendImage(): Bad parameter 'image1'." << std::endl;
        printImageInfo(image1, "image1");
        return NULL;
    }
    if (!image2 || image2->width <= 0 || image2->height <= 0 || image2->depth != 8 || image2->nChannels != 3) {
        std::cout << "Error in blendImage(): Bad parameter 'image2'." << std::endl;
        printImageInfo(image2, "image2");
        return NULL;
    }
    if (!imageAlphaMask || imageAlphaMask->width <= 0 || imageAlphaMask->height <= 0 || imageAlphaMask->depth != 8 || imageAlphaMask->nChannels != 1) {
        std::cout << "Error in blendImage(): Bad parameter 'imageAlphaMask'." << std::endl;
        printImageInfo(imageAlphaMask, "imageAlphaMask");
        return NULL;
    }
    // Make sure that image1 & image2 & imageAlphaMask are all the same dimensions
    if ( (image1->width != image2->width || image1->width != imageAlphaMask->width) ||
         (image1->height != image2->height || image1->height != imageAlphaMask->height) ) {
        std::cout << "Error in blendImage(): Input images aren't the same dimensions." << std::endl;
        printImageInfo(image1, "image1");
        printImageInfo(image2, "image2");
        printImageInfo(imageAlphaMask, "imageAlphaMask");
        return NULL;
    }

    // Combine the 2 images using the alpha channel mask
    int width = image1->width;
    int height = image1->height;
    IplImage *imageBlended = cvCreateImage(cvSize(width, height), image1->depth, image1->nChannels);
    int widthStep1 = image1->widthStep;
    int widthStep2 = image2->widthStep;
    int widthStepM = imageAlphaMask->widthStep;
    int widthStepBlended = imageBlended->widthStep;
    // Start from the beginning of the image and move through the image, row by row.
    UCHAR *p1 = (UCHAR*)image1->imageData;
    UCHAR *p2 = (UCHAR*)image2->imageData;
    UCHAR *pM = (UCHAR*)imageAlphaMask->imageData;
    UCHAR *pBlended = (UCHAR*)imageBlended->imageData;
    for (int y=0; y < height; y++) {
        for (int x=0; x < width; x++) {
            int m = *pM++;    // Get the alpha mask value (0 to 255) and move to its next pixel.
            int part1R, part1G, part1B;
            int part2R, part2G, part2B;
            int pixel1B = *p1++;    // Get the B value and move to its next pixel.
            int pixel1G = *p1++;    // Get the G value and move to its next pixel.
            int pixel1R = *p1++;    // Get the R value and move to its next pixel.
            // Instead of dividing slowly by 255: multiply image2 with (alpha mask + 1), then divide it by 256.
            // It is similar to dividing by 255, but much faster.
            // But note that if m was 0, it would be slightly off, so m == 0 is treated separately.
            if (m) {
                int f1 = m + 1;
                int pixel2B = *p2++;    // Get the B value and move to its next pixel.
                int pixel2G = *p2++;    // Get the G value and move to its next pixel.
                int pixel2R = *p2++;    // Get the R value and move to its next pixel.
                part2B = ( pixel2B * f1 ) >> 8;
                part2G = ( pixel2G * f1 ) >> 8;
                part2R = ( pixel2R * f1 ) >> 8;
                // Multiply image1 with (255 - alpha mask), then divide it by 256. Similar to dividing by 255.0, but much faster.
                int f2 = 255 - m;
                part1B = ( pixel1B * f2 ) >> 8;
                part1G = ( pixel1G * f2 ) >> 8;
                part1R = ( pixel1R * f2 ) >> 8;
            }
            else {
                // If the alpha mask was 0, then just use image1 and not image2.
                part1B = pixel1B;
                part1G = pixel1G;
                part1R = pixel1R;
                part2B = 0;
                part2G = 0;
                part2R = 0;
                p2 += 3;    // move the pointer to the next pixel.
            }
            // Combine the part of image1 with the part of image2, and then move to the next pixel.
            *pBlended++ = part1B + part2B;
            *pBlended++ = part1G + part2G;
            *pBlended++ = part1R + part2R;
        }
        // Move everything to the next row by adding widthStep and subtracting how much of the row was already processed.
        p1 += widthStep1 - width * 3;    // Move to the next RGB row, ignoring any row padding.
        p2 += widthStep2 - width * 3;    // Move to the next RGB row, ignoring any row padding.
        pM += widthStepM - width;        // Move to the next Greyscale row, ignoring any row padding.
        pBlended += widthStepBlended - width * 3;    // Move to the next RGB row, ignoring any row padding.
    }

    return imageBlended;
}


// Save the given image to a JPG or BMP file, even if its format isn't an 8-bit image, such as a 32bit float image.
int saveImage(const char *filename, const IplImage *image)
{
    int ret = -1;
#ifdef USE_HIGHGUI
    IplImage *image8Bit = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, image->nChannels);    // 8-bit greyscale image.
    if (image8Bit)
        cvConvert(image, image8Bit);    // Convert to an 8-bit image instead of potentially 16,24,32 or 64bit image.
    if (image8Bit)
        ret = cvSaveImage(filename, image8Bit);
    if (image8Bit)
        cvReleaseImage(&image8Bit);
#endif
    return ret;
}

// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
// since cvSaveImage() can only handle 8bit images (not 32bit float images).
void saveFloatMat(const char *filename, const CvMat *srcMat)
{
    //cout << "in Saving Image(" << ((CvMat*)src)->width << "," << ((CvMat*)src)->height << ") '" << filename << "'." << endl;
    if (CV_MAT_CN(srcMat->type) != 1 || CV_MAT_DEPTH(srcMat->type) != 32) {
        LOG("ERROR in saveFloatMat(): Matrix is not single-channel 32-bit float matrix!");
        return;
    }

    // Fill the Matrix's float data as a float image into this temporary image, since it wont be needed after this function.
    IplImage srcIplImg;
    cvGetImage(srcMat, &srcIplImg);
    // Store the float image
    saveFloatImage(filename, &srcIplImg);
}

// Get an 8-bit equivalent of the 32-bit Float Matrix.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* convertMatrixToUcharImage(const CvMat *srcMat)
{
    // Fill the Matrix's float data as a float image into this image.
    IplImage srcIplImg;
    cvGetImage(srcMat, &srcIplImg);

    // Convert the float image into a normal Uchar image.
    return convertFloatImageToUcharImage(&srcIplImg);
}

// Get an 8-bit equivalent of the 32-bit Float image.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* convertFloatImageToUcharImage(const IplImage *srcImg)
{
    IplImage *dstImg = 0;
    if ((srcImg) && (srcImg->width > 0 && srcImg->height > 0)) {

        if (srcImg->nChannels != 1 || srcImg->depth != 32) {
            LOG("ERROR in convertFloatImageToUcharImage(): Image is not single-channel 32-bit float image!");
            return 0;
        }

        // Spread the 32bit floating point pixels to fit within 8bit pixel range.
        double minVal, maxVal;
        cvMinMaxLoc(srcImg, &minVal, &maxVal);

        LOG("FloatImage:(minV=%lf, maxV=%lf)", minVal, maxVal);

        // Deal with NaN and extreme values, since the DFT seems to give some NaN results.
        if (cvIsNaN(minVal) || minVal < -1e30)
            minVal = -1e30;
        if (cvIsNaN(maxVal) || maxVal > 1e30)
            maxVal = 1e30;
        if (maxVal-minVal == 0.0f)
            maxVal = minVal + 0.001;    // remove potential divide by zero errors.

        // Convert the format
        dstImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), 8, 1);
        cvConvertScale(srcImg, dstImg, 255.0 / (maxVal - minVal), - minVal * 255.0 / (maxVal-minVal));
    }
    return dstImg;
}

// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
// since cvSaveImage() can only handle 8bit images (not 32bit float images).
void saveFloatImage(const char *filename, const IplImage *srcImg)
{
#ifdef USE_HIGHGUI
    //LOG("Saving Float Image '%s' (%dx%d)", filename, srcImg->width, srcImg->height);
    if (srcImg->nChannels != 1 || srcImg->depth != 32) {
        LOG("ERROR in saveFloatImage(): Image is not single-channel 32-bit float image!");
        return;
    }
    IplImage *byteImg = convertFloatImageToUcharImage(srcImg);
    cvSaveImage(filename, byteImg);
    cvReleaseImage(&byteImg);
    //cout << "done saveFloatImage()" << endl;
#endif
}

// Draw some text onto an image using printf() format.
void drawText(IplImage *img, CvPoint position, CvScalar color, char *fmt, ...)
{
    // Write the given arguments to 'szMsg' using printf() style formatting.
    va_list marker;
    char szMsg[1024];
    va_start(marker, fmt);
    vsprintf(szMsg, fmt, marker);
    va_end(marker);
    //marker = marker;    // stop warning messages

    // Display the text onto the image.
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3,0.3, 0, 1, CV_AA);
    cvPutText(img, szMsg, position, &font, color);
}

