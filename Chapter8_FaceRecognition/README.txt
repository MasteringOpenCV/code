******************************************************************************
*   Face Recognition using Eigenfaces or Fisherfaces
******************************************************************************
*   by Shervin Emami, 5th Dec 2012
*   http://shervinemami.info/openCV.html
******************************************************************************
*   Ch8 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************


Note: You need OpenCV v2.4.1 or later (from June 2012), otherwise the FaceRecognizer will not compile or run.
And you need atleast 3 Face & Eye detection XML files from OpenCV, as shown below.

----------------------------------------------------------
Building the project using CMake from the command-line:
----------------------------------------------------------
Linux:
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cp $OpenCV_DIR/../data/lbpcascades/lbpcascade_frontalface.xml .
    cp $OpenCV_DIR/../data/haarcascades/haarcascade_eye.xml .
    cp $OpenCV_DIR/../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml .
    cmake -D OpenCV_DIR=$OpenCV_DIR ..
    make 

MacOSX (Xcode):
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cp $OpenCV_DIR/../data/lbpcascades/lbpcascade_frontalface.xml .
    cp $OpenCV_DIR/../data/haarcascades/haarcascade_eye.xml .
    cp $OpenCV_DIR/../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml .
    cmake -G Xcode -D OpenCV_DIR=$OpenCV_DIR ..
    open WebcamFaceRec.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    mkdir Debug
    mkdir Release
    copy %OpenCV_DIR%\..\data\lbpcascades\lbpcascade_frontalface.xml .\Debug\
    copy %OpenCV_DIR%\..\data\lbpcascades\lbpcascade_frontalface.xml .\Release\
    copy %OpenCV_DIR%\..\data\haarcascades\haarcascade_eye.xml .\Debug\
    copy %OpenCV_DIR%\..\data\haarcascades\haarcascade_eye.xml .\Release\
    copy %OpenCV_DIR%\..\data\haarcascades\haarcascade_eye_tree_eyeglasses.xml .\Debug\
    copy %OpenCV_DIR%\..\data\haarcascades\haarcascade_eye_tree_eyeglasses.xml .\Release\
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start WebcamFaceRec.sln 

    
----------------------------------------------------------
Running the project:
----------------------------------------------------------
Just execute "WebcamFaceRec".

If it says it can't find a Haar or LBP cascade XML file, copy those XML files from the OpenCV "data" folder to your current folder.

Warning for Visual Studio users: If you run the program directly in Visual Studio (eg: by clicking on "Debug->Start Without Debugging"), then Visual Studio will default to setting the "current folder" as the parent folder instead of the folder with "WebcamFaceRec.exe". So you might need to move or copy the XML file from the Debug / Release folder to the parent folder for it to run directly in Visual Studio. Or adjust your project properties so that it executes the program in the project output folder instead of the solution folder.

