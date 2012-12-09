******************************************************************************
*   Cartoonifier, for Desktop or Android.
******************************************************************************
*   by Shervin Emami, 5th Dec 2012
*   http://shervinemami.info/openCV.html
******************************************************************************
*   Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************

This folder contains 2 Cartoonifier projects, performing the same task:
"Cartoonifier_Desktop": A desktop program (works on Windows, Mac, Linux, etc).
"Cartoonifier_Android": An Android app (requires Android 3.0 SDK or higher, and an Android 2.2 device or higher).

Cartoonifier_Android is a GUI wrapper, so it accesses some of the same C/C++ files in the Cartoonifier_Desktop folder:
    "cartoon.cpp" & "cartoon.h": all of the Cartoonifier image processing.
    "ImageUtils_v0.7.cpp" & "ImageUtils.h": useful functions for debugging OpenCV code.

Each project has code for its user interface and the project files in its own folder:

"Cartoonifier_Desktop" just uses the file "main_desktop.cpp" for its OpenCV user interface on a desktop.
It includes a CMake project file to allow building with different compilers & versions for Windows, Mac, Linux, etc.

"Cartoonifier_Android" has an Android folder tree for its Android user interface, including:
    Java files in the "src" folder, 
    C/C++ NDK files in the "jni" folder,
    app resources in the "res" folder.
It includes an Eclipse project, that you can use for Android cross-development on Windows, Mac & Linux.


----------------------------------------------------------
Building the Cartoonifier_Desktop project using CMake from the command-line:
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
----------------------------------------------------------
Just execute "Cartoonifier_Desktop".


----------------------------------------------------------
Building & Running the Cartoonifier_Android project:
----------------------------------------------------------
Follow the steps recommended in Chapter 1 of the book.

