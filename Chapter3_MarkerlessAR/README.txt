******************************************************************************
*   Marker-less Augmented Reality project sample
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************


To build this app use the CMake to generate project files for your IDE, then build the project in your IDE.
NOTE: You will need to have OpenCV built with OpenGL support in order to run the demo (prebuilt versions of OpenCV don't support OpenGL).

----------------------------------------------------------
How to enable OpenGL Support in OpenCV
----------------------------------------------------------
 * Linux:   Execute "sudo apt-get install libgtkglext1 libgtkglext1-dev" first.
 * MacOSX:  Install QT4 and then configure OpenCV with QT and OpenGL.
 * Windows: Enable WITH_OPENGL=YES flag when building OpenCV to enable OpenGL support.

----------------------------------------------------------
Building the project using CMake from the command-line:
----------------------------------------------------------
Linux:
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -D OpenCV_DIR=$OpenCV_DIR ..
    make 

MacOSX (Xcode):
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -G Xcode -D OpenCV_DIR=$OpenCV_DIR ..
    open EXAMPLE_MARKERLESS_AR.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start EXAMPLE_MARKERLESS_AR.sln 

    
----------------------------------------------------------
Running the project:
----------------------------------------------------------
Just execute "EXAMPLE_MARKERLESS_AR".

