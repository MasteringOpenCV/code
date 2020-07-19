******************************************************************************
*   Cartoonifier, for Desktop or Android.
******************************************************************************
*   by Shervin Emami, 5th Dec 2012
*   http://shervinemami.info/openCV.html
******************************************************************************
*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   2nd Edition.
*   Copyright Packt Publishing 2016.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************

This folder contains the Cartoonifier project, that can be used on desktop
(works on Windows, Mac, Linux, etc) or embedded (works on Raspberry Pi, etc).

The file "main.cpp" is for the OpenCV user interface and "cartoon.cpp" is for
the image processing.
It includes a CMake project file to allow building with different compilers &
versions for Windows, Mac, Linux, etc.


----------------------------------------------------------
Building the Cartoonifier project using CMake from the command-line:
(From the "Cartoonifier" folder):
----------------------------------------------------------
Embedded (Raspberry Pi, etc):
    Follow the steps recommended in Chapter 1 of the book.

Desktop Linux:
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
    open Cartoonifier.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start Cartoonifier.sln 

    
----------------------------------------------------------
Running the project:
(From the "Cartoonifier" folder):
----------------------------------------------------------
Just execute "Cartoonifier", such as "./Cartoonifier" in Linux.

