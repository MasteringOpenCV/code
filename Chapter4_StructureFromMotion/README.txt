******************************************************************************
*   Exploring Structure from Motion using OpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************

NOTE: To build this project, you will need the Point Clouds Library (PCL) from "http://pointclouds.org/".

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
    open ExploringSfMWithOpenCV.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start ExploringSfMWithOpenCV.sln 

    
----------------------------------------------------------
Running the project:
----------------------------------------------------------
    ExploringSfMExec <path_to_images>    

    
----------------------------------------------------------
Files
----------------------------------------------------------
SfM Library
    Common.*                - Common data structures and utility functions
    Triangulation.*         - Triangulation functions
    FeatureMatching.*       - Feature matching functions
    FindCameraMatrices.*    - Finding camera matrices functions
    IDistance.h             - Abstract class for SfM methods
    Distance.*              - SfM class for 2-view reconstruction
    MultiCameraDistace.*    - SfM abstract class for n-view reconstruction
    MultiCameraPnP.*        - SfM class for n-view using camera resection (PnP)
    
SfM Application
    main.cpp                - Main program
    Visualization.*         - Visualization of result point cloud using PCL
