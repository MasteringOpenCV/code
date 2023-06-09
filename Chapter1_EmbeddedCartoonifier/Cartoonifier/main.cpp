/*****************************************************************************
*   Cartoonifier.cpp: Main GUI for the Cartoonifier application.
*   Converts a real-life camera stream to look like a cartoon.
*   This file is for a desktop or embedded Linux executable.
******************************************************************************
*   by Shervin Emami, 8th Aug 2016 (shervin.emami@gmail.com)
*   http://www.shervinemami.info/
******************************************************************************
*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects", 2nd Edition.
*   Copyright Packt Publishing 2016.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/


// Try to set the camera resolution. Note that this only works for some cameras on
// some computers and only for some drivers, so don't rely on it to work!
const int DEFAULT_CAMERA_WIDTH = 640;
const int DEFAULT_CAMERA_HEIGHT = 480;

const char * DEFAULT_CAMERA_NUMBER = "0";

const int NUM_STICK_FIGURE_ITERATIONS = 40; // Sets how long the stick figure face should be shown for skin detection.

const char *windowName = "Cartoonifier";   // Name shown in the GUI window.


// Set to true if you want to see line drawings instead of paintings.
bool m_sketchMode = true;
// Set to true if you want to change the skin color of the character to an alien color.
bool m_alienMode = false;
// Set to true if you want an evil "bad" character instead of a "good" character.
bool m_evilMode = false;
// Set to true if you want to see many windows created, showing various debug info. Set to 0 otherwise.
bool m_debugMode = false;



#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>           // For isdigit()

// Include OpenCV's C++ Interface
#include "opencv2/opencv.hpp"

// Include the rest of our code!
//#include "detectObject.h"       // Easily detect faces or eyes (using LBP or Haar Cascades).
#include "cartoon.h"            // Cartoonify a photo.
#include "ImageUtils.h"      // Shervin's handy OpenCV utility functions.
#include "fps_timer.hpp"          // FPS timer by Jason Saragih.

using namespace cv;
using namespace std;

int m_stickFigureIterations = 0;  // Draws a stick figure outline for where the user's face should be.

#if !defined VK_ESCAPE
    #define VK_ESCAPE 0x1B      // Escape character (27)
#endif



// Get access to the webcam or video source. cameraNumber should be a number
// (eg: "0" or "1") but can also be a video file or stream URL.
void initCamera(VideoCapture &videoCapture, char* cameraNumber)
{
    // First try to access to the camera as a camera number such as 0
    try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
        if ( isdigit(cameraNumber[0]) ) {
            videoCapture.open(atoi(cameraNumber));
        }
    } catch (cv::Exception &e) {}

    if ( !videoCapture.isOpened() ) {
        // Also try to access to the camera as a video file or URL.
        try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
            videoCapture.open(cameraNumber);
        } catch (cv::Exception &e) {}

        if ( !videoCapture.isOpened() ) {
            cerr << "ERROR: Could not access the camera " << cameraNumber << " !" << endl;
            exit(1);
        }
    }
    cout << "Loaded camera " << cameraNumber << endl;
}


// Keypress event handler. Note that it should be a 'char' and not an 'int' to better support Linux.
void onKeypress(char key)
{
    switch (key) {
    case 's':
        m_sketchMode = !m_sketchMode;
        cout << "Sketch / Paint mode: " << m_sketchMode << endl;
        break;
    case 'a':
        m_alienMode = !m_alienMode;
        cout << "Alien / Human mode: " << m_alienMode << endl;
        // Display a stick figure outline when alien skin is enabled,
        // so the user puts their face in the correct place.
        if (m_alienMode) {
            m_stickFigureIterations = NUM_STICK_FIGURE_ITERATIONS;
        }
        break;
    case 'e':
        m_evilMode = !m_evilMode;
        cout << "Evil / Good mode: " << m_evilMode << endl;
        break;
    case 'd':
        m_debugMode = !m_debugMode;
        cout << "Debug mode: " << m_debugMode << endl;
        break;
    }
}


int main(int argc, char *argv[])
{
    cout << "Cartoonifier, by Shervin Emami (www.shervinemami.info), June 2016." << endl;
    cout << "Converts real-life images to cartoon-like images." << endl;
    cout << "Compiled with OpenCV version " << CV_VERSION << endl;
    cout << "usage:   " << argv[0] << " [[camera_number] desired_width desired_height ]" << endl;
    cout << "default: " << argv[0] << " " << DEFAULT_CAMERA_NUMBER << " " << DEFAULT_CAMERA_WIDTH << " " << DEFAULT_CAMERA_HEIGHT << endl;
    cout << endl;

    cout << "Keyboard commands (press in the GUI window):" << endl;
    cout << "    Esc:  Quit the program." << endl;
    cout << "    s:    change Sketch / Paint mode." << endl;
    cout << "    a:    change Alien / Human mode." << endl;
    cout << "    e:    change Evil / Good character mode." << endl;
    cout << "    d:    change debug mode." << endl;
    cout << endl;

    char *cameraNumber = (char*)DEFAULT_CAMERA_NUMBER;
    int desiredCameraWidth = DEFAULT_CAMERA_WIDTH;
    int desiredCameraHeight = DEFAULT_CAMERA_HEIGHT;

    // Allow the user to specify a camera number, since not all computers will be the same camera number.
    int a = 1;
    if (argc > a) {
        cameraNumber = argv[a];
        a++;    // Next arg

        // Allow the user to specify camera resolution.
        if (argc > a) {
            desiredCameraWidth = atoi(argv[a]);
            a++;    // Next arg

            if (argc > a) {
                desiredCameraHeight = atoi(argv[a]);
                a++;    // Next arg
            }
        }
    }

    // Get access to the camera.
    VideoCapture camera;
    initCamera(camera, cameraNumber);

    // Try to set the camera resolution. Note that this only works for some cameras on
    // some computers and only for some drivers, so don't rely on it to work!
    camera.set(CV_CAP_PROP_FRAME_WIDTH, desiredCameraWidth);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, desiredCameraHeight);

    // Create a GUI window for display on the screen.
    namedWindow(windowName, WINDOW_NORMAL); // Fullscreen windows must be _NORMAL
    // Make our window fullscreen.
    setWindowProperty(windowName, WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    // Keep track of the recent FPS status.
    fps_timer timer;

    // Run forever, until the user hits Escape to "break" out of this loop.
    while (true) {

        // Grab the next camera frame. Note that you can't modify camera frames.
        Mat cameraFrame;
        camera >> cameraFrame;
        if( cameraFrame.empty() ) {
            cerr << "ERROR: Couldn't grab the next camera frame." << endl;
            exit(1);
        }

        Mat displayedFrame = Mat(cameraFrame.size(), CV_8UC3);

        // Use debug type 2 (for desktop) if debug mode is enabled.
        int debugType = 0;
        if (m_debugMode)
            debugType = 2;

        // Run the cartoonifier filter using the selected mode.
        cartoonifyImage(cameraFrame, displayedFrame, m_sketchMode, m_alienMode, m_evilMode, debugType);

        // Show a stick-figure outline of a face for a short duration, so the user knows where to put their face.
        if (m_stickFigureIterations > 0) {
            drawFaceStickFigure(displayedFrame);
            m_stickFigureIterations--;
        }

        // Show the current FPS, displayed to the text console
        timer.increment();
        if (timer.fnum == 0) {
            double fps;
            if (timer.fps < 1.0f)
                fps = timer.fps;                // FPS is a fraction
            else
                fps = (int)(timer.fps + 0.5f);  // FPS is a large number
            cout << fps << " FPS" << endl;
        }

        imshow(windowName, displayedFrame);

        // IMPORTANT: Wait for atleast 20 milliseconds, so that the image can be displayed on the screen!
        // Also checks if a key was pressed in the GUI window. Note that it should be a "char" to support Linux.
        char keypress = waitKey(20);  // This is needed if you want to see anything!
        if (keypress == VK_ESCAPE) {   // Escape Key
            // Quit the program!
            break;
        }
        // Process any other keypresses.
        if (keypress > 0) {
            onKeypress(keypress);
        }

    }//end while

    return EXIT_SUCCESS;
}
