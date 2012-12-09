******************************************************************************
*   Automatic Number Plate Recognition using SVM and Neural Networks
******************************************************************************
*   by David Millán Escrivá, 5th Dec 2012
*   http://blog.damiles.com
******************************************************************************
*   Ch5 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************

This code is sample code to understand how automatic license plate recognition (ANPR) works. It is not for production tasks.
You can use this code as a sample & guide to create your own custom ANPR or OCR applications.


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
    open ANPR.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start ANPR.sln 

    
----------------------------------------------------------
Running the project:
----------------------------------------------------------
    ANPR test/2715DTZ.jpg

You can choose other images that are in the test folder or other images that contain a spanish license plate taken from 2 to 3 meters.
There are also some UNIX Bash scripts in the "utils" folder for Linux or Mac, that need Cygwin to run on Windows.
