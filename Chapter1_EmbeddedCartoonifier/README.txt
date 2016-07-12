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

This folder contains 2 Cartoonifier projects, performing the same task:
"Cartoonifier_Desktop": A desktop program (works on Windows, Mac, Linux, etc).
"Cartoonifier_Embedded": An embedded program (works on Raspberry Pi 1 or higher).

Each project has code for its user interface and the project files in its own folder:

"Cartoonifier_Desktop" just uses the file "main_desktop.cpp" for its OpenCV user interface on a desktop.
It includes a CMake project file to allow building with different compilers & versions for Windows, Mac, Linux, etc.


----------------------------------------------------------
Building the Cartoonifier_Desktop project using CMake from the command-line:
(From the "Cartoonifier_Desktop" folder):
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
    open Cartoonifier_Desktop.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start Cartoonifier_Desktop.sln 

    
----------------------------------------------------------
Running the project:
(From the "Cartoonifier_Desktop" folder):
----------------------------------------------------------
Just execute "Cartoonifier_Desktop".


----------------------------------------------------------
Building & Running the Cartoonifier_Embedded project:
----------------------------------------------------------
Follow the steps recommended in Chapter 1 of the book.

